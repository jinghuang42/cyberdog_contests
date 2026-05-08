import toml
import time
import lcm
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from lcm_status import LcmStatus
import sys
import math
import rclpy
from y_xyz import LCMListener
from head_tof_func import HeadTofSubscriber,get_head_tof_node
from scan import ROSListener
from rclpy.executors import MultiThreadedExecutor
from threading import Thread   
from control import start_camera,deactivate_camera,activate_camera
import threading
from rgb import ImageSubscriber,image_data_func

#发送指令的类，这个写成按照需要发送指令，如果一直发送会存在问题。
class Robot_Ctrl(object):
    def __init__(self):
        self.lc = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.msg = robot_control_cmd_lcmt()
        self.steps = toml.load("./toml/usergait.toml")
        self.life_count = 0
    def update_and_publish(self,num):
        self.msg.mode = self.steps["step"][num]["mode"]
        self.msg.gait_id = self.steps["step"][num]["gait_id"]
        self.msg.contact = self.steps["step"][num]["contact"]
        self.msg.value = self.steps["step"][num]["value"]
        self.msg.duration = self.steps["step"][num]["duration"]
        self.msg.life_count = self.life_count % 128
        self.life_count+=1
        for i in range(3):
            self.msg.vel_des[i] = self.steps["step"][num]["vel_des"][i]
            self.msg.rpy_des[i] = self.steps["step"][num]["rpy_des"][i]
            self.msg.pos_des[i] = self.steps["step"][num]["pos_des"][i]
            self.msg.acc_des[i] = self.steps["step"][num]["acc_des"][i]
            self.msg.acc_des[i + 3] = self.steps["step"][num]["acc_des"][i + 3]
            self.msg.foot_pose[i] = self.steps["step"][num]["foot_pose"][i]
            self.msg.ctrl_point[i] = self.steps["step"][num]["ctrl_point"][i]
        for i in range(2):
            self.msg.step_height[i] = self.steps["step"][num]['step_height'][i]
        self.lc.publish("robot_control_cmd", self.msg.encode())
    def turn_left_or_right(self,cur_angle,tar_angle):
        ang_diff=tar_angle-cur_angle
        if abs(ang_diff)<2:
            return False
        else:
            if -180<ang_diff<0:
                self.update_and_publish(3)  #右转
            else:
                self.update_and_publish(2)  #左转
            return True
    def turn_left_or_right_slow(self,cur_angle,tar_angle):
        ang_diff=tar_angle-cur_angle
        if abs(ang_diff)<1:
            return False
        else:
            if -180<ang_diff<0:
                self.update_and_publish(10)
            else:
                self.update_and_publish(9)
            return True
    # def align(self,scan_data):
    #     # print(f"居中对齐中：{scan_data[0]-scan_data[2]}")
    #     # if scan_data[0] or scan_data[2] ==float("inf"):
    #     #     return False
    #     if abs(scan_data[0]-scan_data[2])<0.02:
    #         return False
    #     else:
    #         if scan_data[0]>scan_data[2]:
    #             # print("左移")
    #             self.update_and_publish(5)
    #         else:
    #             self.update_and_publish(6)
    #             # print("右移")
    #         return True
    def align(self,scan_data,mv_data=0):
        # print(f"居中对齐中：{scan_data[0]-scan_data[2]}")
        # if scan_data[0] or scan_data[2] ==float("inf"):
        #     return False
        if abs(scan_data[0]+mv_data-scan_data[2])<0.02:
            return False
        else:
            if scan_data[0]+mv_data>scan_data[2]:
                # print("左移")
                self.update_and_publish(5)
            else:
                self.update_and_publish(6)
                # print("右移")
            return True
    def align2(self,scan_data):
        # print(f"检测是否进入赛道：{scan_data[0]+scan_data[2]}")
        return scan_data[0]+scan_data[2]>1.4
        #scan_node.get_scan()     0.905

    def align_louti(self,scan_data):
        # print(f"居中对齐中：{abs(scan_data[0]-scan_data[2])}")
        if abs(scan_data[0]-0.12-scan_data[2])<0.02:
            return False
        else:
            if scan_data[0]-0.12>scan_data[2]:
                self.update_and_publish(5)
            else:
                self.update_and_publish(6)
            return True
        
    def rpy_fix_right(self,scan_node,cur_angle,tar_angle):
        # print(f"角度补偿中：{scan_node.get_scan()[2]/scan_node.get_scan()[4]}")
        if scan_node.get_scan()[2] and scan_node.get_scan()[4]!=float('inf'):
            while scan_node.get_scan()[2]/scan_node.get_scan()[4] < 0.907 or scan_node.get_scan()[2]/scan_node.get_scan()[4] >0.91:
                if scan_node.get_scan()[2]/scan_node.get_scan()[4] > 0.91:
                    self.update_and_publish(27)
                elif scan_node.get_scan()[2]/scan_node.get_scan()[4] < 0.907:
                    self.update_and_publish(28)
            return tar_angle-cur_angle.y
        else:
            while scan_node.get_scan()[0]/scan_node.get_scan()[3] < 0.907 or scan_node.get_scan()[0]/scan_node.get_scan()[3] > 0.91:
                if scan_node.get_scan()[0]/scan_node.get_scan()[3] > 0.91:
                    self.update_and_publish(28)
                elif scan_node.get_scan()[0]/scan_node.get_scan()[3] <0.907:
                    self.update_and_publish(27)
            return tar_angle-cur_angle.y
    def rpy_fix_left(self,scan_node,cur_angle,tar_angle):
        # print(f"角度补偿中：{scan_node.get_scan()[2]/scan_node.get_scan()[4]}")
        if scan_node.get_scan()[0] and scan_node.get_scan()[3]!=float('inf'):
            while scan_node.get_scan()[0]/scan_node.get_scan()[3] < 0.907 or scan_node.get_scan()[0]/scan_node.get_scan()[3] > 0.91:
                if scan_node.get_scan()[0]/scan_node.get_scan()[3] > 0.91:
                    self.update_and_publish(28)
                elif scan_node.get_scan()[0]/scan_node.get_scan()[3] <0.907:
                    self.update_and_publish(27)
            return tar_angle-cur_angle.y


        else:
            while scan_node.get_scan()[2]/scan_node.get_scan()[4] < 0.907 or scan_node.get_scan()[2]/scan_node.get_scan()[4] >0.91:
                if scan_node.get_scan()[2]/scan_node.get_scan()[4] > 0.91:
                    self.update_and_publish(27)
                elif scan_node.get_scan()[2]/scan_node.get_scan()[4] < 0.907:
                    self.update_and_publish(28)
            return tar_angle-cur_angle.y



def main():
    #性能修改部分1.对于scan中的返回结果，使用函数调用，函数调用时进行计算。
    #2.对于tof中的返回值，只有在楼梯的时候才会使用，下楼梯关闭。
    #3.对于相机的返回值，只在帘子前开启节点，然后执行关闭节点，然后开启节点，关闭节点。4
    #4.对于lcm_status的订阅，50Hz，接受时休眠0.02s
    #5.对于y_xyx的订阅，50Hz，接收时休眠0.02s
    #6.对于cmd_lcm的发布，2~500Hz，发布时休眠0.002
    #7.对于ros2的scan的订阅，约为10Hz，接收时休眠0.1s
    #8.对于ros2的tof的订阅，约为10Hz，接收时休眠0.2s
    if len(sys.argv) < 3:
        return
    user_input = int(sys.argv[1])  # 接收命令行参数
    init_angle = int(sys.argv[2])
    #存储腿式里程计的信息
    xyz=None
    rclpy.init(args=None)
    scan_node = ROSListener()
    executor=MultiThreadedExecutor()
    # image_node=ImageProcessor()
    executor.add_node(scan_node)
    # executor.add_node(image_node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    y_xyz_node = LCMListener()
    robot_ctrl = Robot_Ctrl()
    lcm_status = LcmStatus()
    lcm_status.run()
    fix = init_angle-y_xyz_node.y
    fix_cos = math.cos(fix/180*math.pi)
    head_tof_node = get_head_tof_node()
    angles=[0,90,180,270] # 方向
    at_direction=0
    if init_angle in [90,180,270]:
        at_direction=init_angle//90
    def act(x):
        while lcm_status.rec_msg.order_process_bar==100:
            robot_ctrl.update_and_publish(x)
        while lcm_status.rec_msg.order_process_bar<100:
            robot_ctrl.update_and_publish(x)
    def gaotai():
        '''
        上一层高台        
        '''
        print('上高台')

        act(22)
        act(24)
        act(0)
        act(29)
        # act(29)
        # act(29)
        act(0)
    def chugaotai(at_direction):
        '''
        出高台        
        '''
        act(25)
        act(0)
        print('朝前')
        while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,angles[at_direction]):
            continue
        # act(29)
        act(25)
        act(0)
        print('朝前')
        while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,angles[at_direction]):
            continue
        center()
            # act(25)
            # act(0)
    def process_bridge(head_tof_node):
        '''
        独木桥
        '''
        act(0)
        count = 0
        xyz=y_xyz_node.xyz
        while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos > -1.7:
            #print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
            if count == 2:
                # 低速右转
                act(33)
                count=0
            if count == -2:
                # 低速左转
                act(32)
                count=0
            temp = head_tof_node.tof_dumuqiao()
            if temp == 1:
                count+=1
                # 右移
                robot_ctrl.update_and_publish(6)
            elif temp == -1:
                count-=1
                # 左移
                robot_ctrl.update_and_publish(5)
            else:
                # 前进
                robot_ctrl.update_and_publish(7)
        #关闭tof
        executor.remove_node(head_tof_node)
    def louti():
        '''
        爬楼梯
        '''
        act(0)
        act(31)


    def leg_odometer(range,at_direction,gait_num=7):
        '''
        腿式里程计，at_direction表示朝向，0表示超前，1表示朝左，2表示向后，3表示朝右（前后左右以机器狗开机时的朝向为准）
        range表示移动距离
        '''


        
        # xyz=y_xyz_node.xyz
        # while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos < 1.524:
        #     # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
        #     robot_ctrl.update_and_publish(7)
        xyz=y_xyz_node.xyz
        if at_direction==0:
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos< range:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(gait_num)
        if at_direction==1:
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos < range:
                # print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
                robot_ctrl.update_and_publish(gait_num)
        if at_direction==2:
            while y_xyz_node.xyz[0]/fix_cos - xyz[0]/fix_cos > -range:
                # print(f"移动中：{y_xyz_node.xyz[2]/fix_cos-xyz[2]/fix_cos}")
                robot_ctrl.update_and_publish(gait_num)

        if at_direction==3:
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos > -range:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(gait_num)
    def center():
        print('前进并判断是否能剧中')
        while robot_ctrl.align2(scan_node.get_scan()):
            robot_ctrl.update_and_publish(7)
        act(29)
        #居中对齐
        while robot_ctrl.align(scan_node.get_scan()):
            continue
    def turn_left(at_direction,gait_num=7):
        '''
        左转
        '''
        at_direction=at_direction+1
        #行走至离墙面0.4m
        while scan_node.get_scan()[1] > 0.4:
            robot_ctrl.update_and_publish(gait_num)  
        #3.左转到90度位置
        while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,    angles[at_direction]):
            # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
            continue
        return at_direction
    def rotate(angle):
        '''
        旋转机器狗狗到某个角度
        '''
        #旋转
        while abs((y_xyz_node.y+fix+360)%360-angle) > 3:
            robot_ctrl.update_and_publish(20)  
    def guoyuanzhu():
        '''
        过圆柱
        需根据圆柱大小调整不太的转弯速度，vel_des =  [a,b,c]   a线速度，c角速度
        '''
        # 旋转
        rotate(0)
        #后退
        act(38)

        #前进至圆柱前0.2m
        while scan_node.get_scan()[0] > 0.3:
            robot_ctrl.update_and_publish(5)

        #绕圆台
        while abs((y_xyz_node.y+fix+360)%360-140) > 3:
            # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
            robot_ctrl.update_and_publish(21)
        print('绕圆台结束')
         #出圆柱
        act(26)
        #旋转
        rotate(90)
    def curtain(at_direction,bias_value,range ):
        '''
        通过幕布函数 
        bias_value表示居中偏置值
        range表示移动距离
        '''
        while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,angles[at_direction]):
            continue
        while robot_ctrl.align(scan_node.get_scan(),bias_value):
            continue            
        leg_odometer(range,at_direction)

    def all_curtains():
        '''
        通过所有幕布函数
        '''
        #第一块幕布
        print('朝前')
        curtain(at_direction,-0.5,1.1)
        #第二块幕布
        print('朝前')
        curtain(at_direction,0.5,0.9)

        #第三块幕布
        curtain(at_direction,-0.5,0.9)
        print('彩色幕布结束')

    #这种情况下根据上一个指令执行是否完成下发下一个指令。这个心跳包是什么情况，如果狗狗一直在运动，是不是就是不需要心跳包。不是的，狗子在运行过程中需要一直发送心跳包
    #这种情况下是不是应该开一个线程执行指令，还是在原来的线程里面执行指令，如果在原来的线程上执行指令，是否会阻塞，这里是否可以阻塞。可以在原来的线程中继续执行，可以阻塞。
    #存在一个问题，如果在帘子那一关执行判断需要花比较长的时间，心跳包是否还会延续，如果不能延续只能开一个线程执行。
    try:
        #1.起始蹲下，执行站立
        print('起立')
        act(0)
        # time.sleep(1)
        # gaotai()

        # #第1个关卡高台
        if user_input<1:

            #act(0)
            gaotai()
            # louti()
            # 第二层高台
            act(29)
            act(0)
            gaotai()
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,angles[at_direction]):
                continue
            chugaotai(at_direction=at_direction)


        #第2个关卡台阶
        if user_input<2:
            print('台阶')
            # 台阶准备
            leg_odometer(0.45,at_direction)
            act(29)
            # act(0)
            # 台阶
            louti()
            print('台阶结束')
            act(0)
        #第3个关卡独木桥
        if user_input<3:
            print('独木桥')
            # 独木桥准备
            act(29)
            process_bridge(head_tof_node=head_tof_node)
            print('独木桥结束')
        #第4个关卡下台阶与石板路

        #if user_input<4:
        #    print('下台阶和石板路')
        #    print('前进并判断是否能剧中')
        #    while robot_ctrl.align2(scan_node.get_scan()):
        #        robot_ctrl.update_and_publish(7)
        #    leg_odometer(0.2,at_direction,42)
        #    center()
        #    print('朝前')
        #    while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,angles[at_direction]):
        #        continue
        #    leg_odometer(2.7,at_direction,42)
        #    print('下台阶和石板路结束')
        #第5个障碍第1个直角弯
        #if user_input<5:
        #    print('第1个直角弯')
        #    # 转直角弯
        #    at_direction=turn_left(at_direction=at_direction,gait_num=42)
        #    print("第1个直角弯结束，即将进入彩色幕布")
        #第6个障碍彩色幕布
        #if user_input<6:
        #    print('彩色幕布')
        #    # 彩色幕布准备
        #    print('朝前')
        #    while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,angles[at_direction]):
        #        continue
        #    center()
#       #      #通过彩色幕布
        #    all_curtains()

        #第7个障碍第2个直角弯
        #if user_input<7:
        #    print('第2个直角弯')
        #    # 转直角弯
        #    at_direction=turn_left(at_direction=at_direction,gait_num=42)
          
        # 第8个关卡沙地
        #if user_input<8:  
        #    print('沙地')
        #    # 进入沙地
        #    # 腿式里程计前进0.5m
        #    leg_odometer(0.5,at_direction)
        #    print("行走结束，即将进入沙地")
        #    # 沙地
        #    # 居中对齐
        #    center()
        #    print('朝前')
        #    while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,angles[at_direction]):
        #        continue
        #    # 腿式里程计前进1.5m
        #    leg_odometer(1.524,at_direction)
        #    print("沙地结束，即将进入石子路")

        # 第9个关卡石子路
        #if user_input<9:
        #    print('石子路')
        #    center()
        #    leg_odometer(1.83,at_direction)
        #    print("石子路结束，即将上斜坡")

        # 第10个关卡斜坡
        #if user_input<10:
        #    print('斜坡')
        #    leg_odometer(1.5,at_direction)
        #    print("即将达到斜坡顶端")
        #    print("斜坡顶端")
        #    leg_odometer(0.75,at_direction)
        #    print("斜坡顶端结束")
        #    print("开始下斜坡")
        #    leg_odometer(0.95,at_direction)
        #    center()
        #    print('朝前')
        #    while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,angles[at_direction]):
        #        continue
        #    print("下斜坡结束,开始减速带")


        # 第11个关卡减速带
        #if user_input<11:
        #    print('减速带')
        #    leg_odometer(2.2,at_direction)
        #    print("减速带结束，即将进入直角弯")

        # 第12个关卡直角弯
        #if user_input<12:
        #    print('直角弯')
        #    # 转弯准备
        #    print('朝前')
        #    while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,angles[at_direction]):
        #        continue
        #    print("开始直角转弯")
        #    at_direction=turn_left(at_direction)
        # 第13个关卡圆柱
        #if user_input<13:
        #    print('圆柱')
        #    leg_odometer(0.5,at_direction)
        #    center()

            # 圆柱准备

        #    print('朝前')
        #    while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,angles[at_direction]):
        #        continue
        #    while scan_node.get_scan()[1] > 0.20:
        #        robot_ctrl.update_and_publish(7)
        #    print("开始过圆柱")
        #    guoyuanzhu()
        #    print("圆柱结束，即将第二次直角弯")
        #第14个关卡二次直角弯
        #if user_input<14:
            # 第二个直角弯
            # 直角弯准备
        #    print('第二次直角弯')
            # leg_odometer(0.2,at_direction)
        #    print('朝前')
        #    while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,angles[at_direction]):
        #        continue  
        #    center()
          
            # 转直角弯
        #    at_direction=  turn_left(at_direction=at_direction)
            # print("第二次直角弯结束，即将进入高台")

#    #进入第四段
#         #颜色识别，如果是偏向一边，可不可以，如果居中，可不可以。
#         if user_input<14:
#             #走到最后一段
#             while robot_ctrl.align2(scan_node.get_scan()):
#                 robot_ctrl.update_and_publish(14)
#             #角度补偿，很关键
#             for _ in range(8):
#                 fix=robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
#             fix_cos=math.cos(fix/180*math.pi)
            

#             #根据腿式里程计前进50cm
#             xyz=y_xyz_node.xyz
#             while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<0.7:
#                 # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
#                 robot_ctrl.update_and_publish(7)
#             #rpy朝前
#             while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
#                 # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
#                 continue
#             #居中对齐
#             while robot_ctrl.align(scan_node.get_scan()):
#                 continue

        #     #开启realsense
        #     camera_thread = threading.Thread(target=start_camera)
        #     camera_thread.start()
        #     start_time = time.time()
        #     while time.time() - start_time < 25:
        #         robot_ctrl.update_and_publish(0)


        # #第一块布
        #     # 创建并启动子进程
        #     thread1=threading.Thread(target=activate_camera)
        #     thread1.start()
        #     image_node1=ImageSubscriber()
        #     executor.add_node(image_node1)
        #     while image_node1.image is None:
        #         robot_ctrl.update_and_publish(0)
        #     thread11=threading.Thread(target=deactivate_camera)
        #     thread11.start()
        #     executor.remove_node(image_node1)
        #     flag=image_data_func(image_node1.image)
        #     try:
        #         image_node1.destroy_node()
        #     except:
        #         print("已清除image_node1")
        #     if flag:   #左移40cm
        #         xyz=y_xyz_node.xyz
        #         while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos<0.27:
        #             # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
        #             robot_ctrl.update_and_publish(5)
        #     else:               #右移40cm
        #         xyz=y_xyz_node.xyz
        #         while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos>-0.27:
        #             # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
        #             robot_ctrl.update_and_publish(6)

        #     #前进1m
        #     xyz=y_xyz_node.xyz
        #     while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<0.88:
        #         # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
        #         robot_ctrl.update_and_publish(7)
            
        #     # 角度补偿
        #     if flag:
        #         for _ in range(5):
        #             fix = robot_ctrl.rpy_fix_left(scan_node,y_xyz_node,0)
        #         fix_cos=math.cos(fix/180*math.pi)
        #     else:
        #         for _ in range(5):
        #             fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
        #         fix_cos=math.cos(fix/180*math.pi)

        #     #rpy朝前
        #     while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
        #         # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
        #         continue
        #     # 居中对齐
        #     while robot_ctrl.align(scan_node.get_scan()):
        #         continue



        # #第二块布
        #     thread2=threading.Thread(target=activate_camera)
        #     thread2.start()
        #     image_node2=ImageSubscriber()
        #     executor.add_node(image_node2)
        #     while image_node2.image is None:
        #         robot_ctrl.update_and_publish(0)
        #     thread22=threading.Thread(target=deactivate_camera)
        #     thread22.start()
        #     executor.remove_node(image_node2)
        #     flag=image_data_func(image_node2.image)
        #     try:
        #         image_node2.destroy_node()
        #     except:
        #         print("已清除image_node1")
        #     if flag:   #左移40cm
        #         xyz=y_xyz_node.xyz
        #         while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos<0.27:
        #             # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
        #             robot_ctrl.update_and_publish(5)
        #     else:               #右移40cm
        #         xyz=y_xyz_node.xyz
        #         while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos>-0.27:
        #             # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
        #             robot_ctrl.update_and_publish(6)

        #     #前进1m
        #     xyz=y_xyz_node.xyz
        #     while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<0.88:
        #         # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
        #         robot_ctrl.update_and_publish(7)
        #     #rpy朝前
        #     while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
        #         # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
        #         continue
        #     #居中对齐
        #     while robot_ctrl.align(scan_node.get_scan()):
        #         continue




        # #第三块布
        #     thread3=threading.Thread(target=activate_camera)
        #     thread3.start()
        #     image_node3=ImageSubscriber()
        #     executor.add_node(image_node3)
        #     while image_node3.image is None:
        #         robot_ctrl.update_and_publish(0)
        #     thread33=threading.Thread(target=deactivate_camera)
        #     thread33.start()
        #     executor.remove_node(image_node3)
        #     flag=image_data_func(image_node3.image)
        #     try:
        #         image_node3.destroy_node()
        #     except:
        #         print("已清除image_node1")
        #     if flag:   #左移40cm
        #         xyz=y_xyz_node.xyz
        #         while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos<0.27:
        #             # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
        #             robot_ctrl.update_and_publish(5)
        #     else:               #右移40cm
        #         xyz=y_xyz_node.xyz
        #         while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos>-0.27:
        #             # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
        #             robot_ctrl.update_and_publish(6)

        #     #前进1m
        #     xyz=y_xyz_node.xyz
        #     while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<0.88:
        #         # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
        #         robot_ctrl.update_and_publish(7)
        #     #rpy朝前
        #     while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
        #         # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
        #         continue
        #     #居中对齐
        #     while robot_ctrl.align(scan_node.get_scan()):
        #         continue
    except KeyboardInterrupt:
        robot_ctrl.update_and_publish("0")
        while lcm_status.rec_msg.order_process_bar<97:
            robot_ctrl.update_and_publish("0")
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
