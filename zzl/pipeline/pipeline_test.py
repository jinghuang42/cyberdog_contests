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
    def act(x):
        while lcm_status.rec_msg.order_process_bar==100:
            robot_ctrl.update_and_publish(x)
        while lcm_status.rec_msg.order_process_bar<100:
            robot_ctrl.update_and_publish(x)
    def gaotai():
            act(22)
            act(24)
            act(0)
            act(29)
            act(29)
            act(29)
            act(0)
            act(25)
            act(0)
    def process_bridge(head_tof_node):
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
        act(0)
        act(31)
        # act(24)
    #这种情况下根据上一个指令执行是否完成下发下一个指令。这个心跳包是什么情况，如果狗狗一直在运动，是不是就是不需要心跳包。不是的，狗子在运行过程中需要一直发送心跳包
    #这种情况下是不是应该开一个线程执行指令，还是在原来的线程里面执行指令，如果在原来的线程上执行指令，是否会阻塞，这里是否可以阻塞。可以在原来的线程中继续执行，可以阻塞。
    #存在一个问题，如果在帘子那一关执行判断需要花比较长的时间，心跳包是否还会延续，如果不能延续只能开一个线程执行。
    try:
        #1.起始蹲下，执行站立
        print('起立')
        act(0)
        # time.sleep(1)
        
#进入沙地
        if user_input<1:  
            print('开始进入沙地')
            #朝前
            print('朝前')
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                continue
            print('前进并判断是否能剧中')
            while robot_ctrl.align2(scan_node.get_scan()):
                robot_ctrl.update_and_publish(7)


            #角度补偿
            for _ in range(5):
                fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
            fix_cos=math.cos(fix/180*math.pi)  
            #2.前进0.6m，根据腿式里程计
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<0.8:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(7)

#            print('前进并判断是否能剧中')
#            while robot_ctrl.align2(scan_node.get_scan()):
#                robot_ctrl.update_and_publish(7)
#            act(29)


            #居中对齐
            print('居中对齐01.0.8')
            while robot_ctrl.align(scan_node.get_scan()):
                continue

            #2.1.前进0.6m，根据腿式里程计
#            xyz=y_xyz_node.xyz
#            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<0.6:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
#                robot_ctrl.update_and_publish(7)

            #行走至离墙面0.3m
#            while True:
#            while scan_node.get_scan()[1] > 0.3:
#                robot_ctrl.update_and_publish(7)


#开始石子路
        if user_input<2:  
            print('开始进入石子路')
            #朝前
            print('朝前')
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                continue
            print('前进并判断是否能剧中')
            while robot_ctrl.align2(scan_node.get_scan()):
                robot_ctrl.update_and_publish(7)


            #角度补偿
            for _ in range(5):
                fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
            fix_cos=math.cos(fix/180*math.pi)  
            #2.前进0.6m，根据腿式里程计
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<1.5:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(7)
            #居中对齐
            print('居中对齐02.1.5')
            while robot_ctrl.align(scan_node.get_scan()):
                continue
            #行走至离墙面0.3m
#            while scan_node.get_scan()[1] > 0.3:
#                robot_ctrl.update_and_publish(7)  


#开始上斜坡
        if user_input<3:  
            print('开始进入上斜坡')
            #朝前
            print('朝前')
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                continue
            print('前进并判断是否能剧中')
            while robot_ctrl.align2(scan_node.get_scan()):
                robot_ctrl.update_and_publish(7)


            #角度补偿
            for _ in range(5):
                fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
            fix_cos=math.cos(fix/180*math.pi)  
            #2.前进0.6m，根据腿式里程计
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<1.8:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(7)
            #居中对齐
            print('居中对齐03.1.8')
            while robot_ctrl.align(scan_node.get_scan()):
                continue
            #行走至离墙面0.3m
#            while scan_node.get_scan()[1] > 0.3:
#                robot_ctrl.update_and_publish(7)  



#开始下斜坡
        if user_input<4:  
            print('开始进入下斜坡')
            #朝前
            print('朝前')
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                continue
            print('前进并判断是否能剧中')
            while robot_ctrl.align2(scan_node.get_scan()):
                robot_ctrl.update_and_publish(7)


            #角度补偿
            for _ in range(5):
                fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
            fix_cos=math.cos(fix/180*math.pi)  
            #2.前进0.6m，根据腿式里程计
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<3.0:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(7)
            #居中对齐
            print('居中对齐04.3.0')
            while robot_ctrl.align(scan_node.get_scan()):
                continue
            #行走至离墙面0.3m
#            while scan_node.get_scan()[1] > 0.3:
#                robot_ctrl.update_and_publish(7)  



#开始减速带
        if user_input<5:  
            print('开始进入减速带')
            #朝前
            print('朝前')
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                continue
            print('前进并判断是否能剧中')
            while robot_ctrl.align2(scan_node.get_scan()):
                robot_ctrl.update_and_publish(7)


            #角度补偿
            for _ in range(5):
                fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
            fix_cos=math.cos(fix/180*math.pi)  
            #2.前进0.6m，根据腿式里程计
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<3.4:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(7)

    #居中对齐
#            print('居中对齐06.3.6')
#            while robot_ctrl.align(scan_node.get_scan()):
#                continue
            #行走至离墙面0.3m
#            while scan_node.get_scan()[1] > 0.3:
#                robot_ctrl.update_and_publish(7)  


# 第一个直角弯
        if user_input<6:
            # 第一个直角弯
            print('开始第一个直角弯')
            #角度补偿
            for _ in range(5):
                fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)


            #行走至离墙面0.4m
            while scan_node.get_scan()[1] > 0.4:
                robot_ctrl.update_and_publish(7)  

            #3.左转到90度位置
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue




#准备进入圆柱
        if user_input<7:
            print('准备进入圆柱')
            #朝前
            print('朝前')
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
                continue
            print('前进并判断是否能剧中')
            while robot_ctrl.align2(scan_node.get_scan()):
                robot_ctrl.update_and_publish(7)


            #角度补偿
            for _ in range(5):
                fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
            fix_cos=math.cos(fix/180*math.pi)
            #2.前进0.6m，根据腿式里程计
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<0.8:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(7)

#圆柱
        if user_input<8:
            
            #朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue
            #居中
            while robot_ctrl.align(scan_node.get_scan()):
                continue

            #前进至圆柱前0.2m
            while scan_node.get_scan()[1] > 0.20:
                robot_ctrl.update_and_publish(7)

            #旋转
            while abs((y_xyz_node.y+fix+360)%360-270) > 3:
                robot_ctrl.update_and_publish(20)

            #后退

            act(38)

            #前进至圆柱前0.2m
            while scan_node.get_scan()[0] > 0.3:
                robot_ctrl.update_and_publish(5)

            #绕圆台
            while abs((y_xyz_node.y+fix+360)%360-50) > 3:
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                robot_ctrl.update_and_publish(21)
            print('绕圆台结束')

            #出圆柱
            act(26)
            #旋转
            while abs((y_xyz_node.y+fix+360)%360-0) > 3:
                robot_ctrl.update_and_publish(20)


            print('出圆柱结束')


#准备出圆柱
        if user_input<9:
            print('准备出圆柱')
            #朝前
            print('朝前')
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                continue
            print('前进并判断是否能剧中')
            while robot_ctrl.align2(scan_node.get_scan()):
                robot_ctrl.update_and_publish(7)


            #角度补偿
            for _ in range(5):
                fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
            fix_cos=math.cos(fix/180*math.pi)
            #2.前进0.6m，根据腿式里程计
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<1.0:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(7)
#            #居中对齐
#            print('居中对齐08.1.2')
#            while robot_ctrl.align(scan_node.get_scan()):
#                continue

#前进
#        if user_input<6:
#            print('前进')
#            #朝前
#            print('朝前')
#            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
#                continue
#            print('前进并判断是否能剧中')
#            while robot_ctrl.align2(scan_node.get_scan()):
#                robot_ctrl.update_and_publish(7)


#            #角度补偿
#            for _ in range(5):
#                fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
#            fix_cos=math.cos(fix/180*math.pi)
#            #2.前进0.6m，根据腿式里程计
#            xyz=y_xyz_node.xyz
#            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<1.2:
#                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
#                robot_ctrl.update_and_publish(7)






# 第2个直角弯
#        if user_input<9:
#            # 第2个直角弯
#            print('第2个直角弯')
#            #角度补偿
#            for _ in range(5):
#                fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)


            #行走至离墙面0.4m
#            while scan_node.get_scan()[1] > 0.4:
#                robot_ctrl.update_and_publish(7)  

            #3.左转到90度位置
#            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
#                continue


# 楼梯前准备
#        if user_input<10:

            #朝前
#            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
#                continue
#            print('前进并判断是否能剧中')
#            while robot_ctrl.align2(scan_node.get_scan()):
#                robot_ctrl.update_and_publish(7)
#            act(29)
               # 剧中对齐
#            while robot_ctrl.align(scan_node.get_scan()):
#                continue  

# 上楼梯
#        if user_input<11:

#            print('开始楼梯')
#            louti()

            # 上台阶

       #上楼梯
#         if user_input<11:
#            while robot_ctrl.align2(scan_node.get_scan()):
#                robot_ctrl.update_and_publish(7)
            #楼梯对齐
#            act(36)

#            while robot_ctrl.align_louti(scan_node.get_scan()):
#                continue

            #楼梯对齐
#            while scan_node.get_scan()[1]>0.76:
                # print(scan_node.get_scan()[1])
#                robot_ctrl.update_and_publish(39)

            #站立
#            act(0)

            #为独木桥开启tof
#            head_tof_node=HeadTofSubscriber()
#            executor.add_node(head_tof_node)

            #自定义上楼梯
#            act(31)

            
            #站立
#            act(0)

            
# 过独木桥
#        if user_input<12:
# 独木桥
            # act(0)
#            process_bridge(head_tof_node=head_tof_node)
# 下台阶和石板路
#        if user_input<13:
            # 下台阶和石板路
#            print('下台阶和石板路')
#            print('前进并判断是否能剧中')
#            while robot_ctrl.align2(scan_node.get_scan()):
#                robot_ctrl.update_and_publish(7)
#            xyz=y_xyz_node.xyz
#            while y_xyz_node.xyz[0]/fix_cos - xyz[0]/fix_cos > -0.2:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
#                robot_ctrl.update_and_publish(42)
            # 剧中对齐
#            print('剧中对齐')
#            while robot_ctrl.align(scan_node.get_scan()):
#                continue  
            #朝前
#            print('朝前')

            # 正式代码，从独木桥结束，朝前为180
#            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,180):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
#                continue

            # #2.前进0.6m，根据腿式里程计
#            xyz=y_xyz_node.xyz
#            while y_xyz_node.xyz[0]/fix_cos - xyz[0]/fix_cos > -2.7:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
#                robot_ctrl.update_and_publish(42)

# 第三个直角弯
#        if user_input<14:
            #第三个直角弯
            #行走至离墙面0.4m
#            print('行走至离墙面0.4m')
#            while scan_node.get_scan()[1] > 0.4:
#                robot_ctrl.update_and_publish(42) 
#            print('石板路结束') 

            # # 正式代码角度为270
#            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
#                continue

#            print('前进并判断是否能剧中')
#            while robot_ctrl.align2(scan_node.get_scan()):
#                robot_ctrl.update_and_publish(7)
#            act(29)

            # 剧中对齐
#            while robot_ctrl.align(scan_node.get_scan()):
#                continue  
#            #朝前
#            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,270):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
#                continue

#通过幕布
#        if user_input<15:
            #幕布
#            print('通过第一块幕布')
#            while robot_ctrl.align(scan_node.get_scan(),-0.5):
#                continue
            #朝前
#            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
#                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
#                continue
#            xyz=y_xyz_node.xyz
#            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos > 0.6:
                # print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
#                robot_ctrl.update_and_publish(7)


    # 继续第二段代码...

#            print('通过第二块幕布')
#            while robot_ctrl.align(scan_node.get_scan(),0.5):
#                continue
            #朝前
#            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,270):
#                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
#                continue
#            xyz=y_xyz_node.xyz
#            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos > 0.8:
                # print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
#                robot_ctrl.update_and_publish(7)


#            print('通过第三块幕布')
#            while robot_ctrl.align(scan_node.get_scan(),-0.5):
#                continue
            #朝前
#            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,270):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
#                continue
#            xyz=y_xyz_node.xyz
#            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos > -0.8:
                # print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
#                robot_ctrl.update_and_publish(7)
            #行走至离墙面0.4m
#            while scan_node.get_scan()[1] > 0.4:
#                robot_ctrl.update_and_publish(7)  
            # # 正式代码角度为0
#            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
#                continue
#            while True:
#                act(0)




    except KeyboardInterrupt:
        robot_ctrl.update_and_publish("0")
        while lcm_status.rec_msg.order_process_bar<97:
            robot_ctrl.update_and_publish("0")
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
