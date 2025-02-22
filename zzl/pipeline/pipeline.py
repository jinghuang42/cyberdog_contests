import toml
import time
import lcm
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from lcm_status import LcmStatus
import sys
import math
import rclpy
from y_xyz import LCMListener
from head_tof_func import HeadTofSubscriber
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
    def align(self,scan_data):
        # print(f"居中对齐中：{scan_data[0]-scan_data[2]}")
        # if scan_data[0] or scan_data[2] ==float("inf"):
        #     return False
        if abs(scan_data[0]-scan_data[2])<0.02:
            return False
        else:
            if scan_data[0]>scan_data[2]:
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
    def act(x):
        while lcm_status.rec_msg.order_process_bar==100:
            robot_ctrl.update_and_publish(x)
        while lcm_status.rec_msg.order_process_bar<100:
            robot_ctrl.update_and_publish(x)
        
    #这种情况下根据上一个指令执行是否完成下发下一个指令。这个心跳包是什么情况，如果狗狗一直在运动，是不是就是不需要心跳包。不是的，狗子在运行过程中需要一直发送心跳包
    #这种情况下是不是应该开一个线程执行指令，还是在原来的线程里面执行指令，如果在原来的线程上执行指令，是否会阻塞，这里是否可以阻塞。可以在原来的线程中继续执行，可以阻塞。
    #存在一个问题，如果在帘子那一关执行判断需要花比较长的时间，心跳包是否还会延续，如果不能延续只能开一个线程执行。
    try:
        #1.起始蹲下，执行站立
        act(0)

        #直角转弯1
        if user_input<1:  
            #角度补偿
            for _ in range(5):
                fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
            fix_cos=math.cos(fix/180*math.pi)  
            #2.前进0.6m，根据腿式里程计
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<0.6:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(7)
            #行走至离墙面0.4m
            while scan_node.get_scan()[1] > 0.4:
                robot_ctrl.update_and_publish(7)  
            
            #3.左转到90度位置
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue

    #进入第一赛段
        #沙地
        if user_input<2:
            #根据雷达判断进入第一阶段赛段
            while robot_ctrl.align2(scan_node.get_scan()):
                robot_ctrl.update_and_publish(7)
            #前进30cm进入赛道
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos < 0.2:
                # print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
                robot_ctrl.update_and_publish(7)

            #朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue

            #居中对齐
            while robot_ctrl.align(scan_node.get_scan()):
                continue

            #前进1.5m通过沙地
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos < 1.4:
                # print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
                robot_ctrl.update_and_publish(7)



        #石子路
        if user_input<3:
            #朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
                continue

            #居中对齐
            while robot_ctrl.align(scan_node.get_scan()):
                continue

            #6前进，根据腿式里程计判断，前进1.5米通过石子路
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos < 1.45:
                # print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
                robot_ctrl.update_and_publish(19)

            
        #上斜坡
        if user_input<4:
            #上0.3m对齐
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos < 0.3:
                # print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
                robot_ctrl.update_and_publish(7)

            #在斜坡上朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue
            #居中
            while robot_ctrl.align(scan_node.get_scan()):
                continue

            #6前进，根据腿式里程计判断，前进1.4米到斜坡顶端
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos < 1.3:
                # print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
                robot_ctrl.update_and_publish(7)

            
            
        #斜坡顶端
        if user_input<5:
            #朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue

            #9.前进，根据腿式里程计判断，前进0.4米到边缘
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos < 0.4:
                # print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
                robot_ctrl.update_and_publish(7)

            
        #下斜坡   
        if user_input<6:
            #下1.4m对齐
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos < 1.35:
                # print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
                robot_ctrl.update_and_publish(7)

            #rpy超前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue

            #居中
            while robot_ctrl.align(scan_node.get_scan()):
                continue

            #11.前进，根据腿式里程计判断，前进2米到直角转弯
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos < 1.2:
                # print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
                robot_ctrl.update_and_publish(7)

            #rpy超前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue

            

        #直角转弯2
        if user_input<7:
            #行走至离墙面0.2m
            while scan_node.get_scan()[1] > 0.4:
                robot_ctrl.update_and_publish(7)

            

            #转弯
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,180):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue
            
    #进入第二段
        #减速带
        if user_input<8:

            #根据雷达前进至第二赛道
            while robot_ctrl.align2(scan_node.get_scan()):
                robot_ctrl.update_and_publish(7)

            act(23)

            #角度补偿
            for _ in range(3):
                fix=robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,180)
            fix_cos=math.cos(fix/180*math.pi)

            #朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,180):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue

            #居中
            while robot_ctrl.align(scan_node.get_scan()):
                continue
            #前进2.8通过减速带
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos - xyz[0]/fix_cos > -2.3:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(19)

            
#圆柱
        if user_input<9:
            
            #朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,180):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue
            #居中
            while robot_ctrl.align(scan_node.get_scan()):
                continue

            #前进至圆柱前0.2m
            while scan_node.get_scan()[1] > 0.20:
                robot_ctrl.update_and_publish(7)
            
            #旋转
            while abs((y_xyz_node.y+fix+360)%360-90) > 3:
                robot_ctrl.update_and_publish(20)

            #后退
            
            act(38)

            #前进至圆柱前0.2m
            while scan_node.get_scan()[0] > 0.3:
                robot_ctrl.update_and_publish(5)

            #绕圆台
            while abs((y_xyz_node.y+fix+360)%360-230) > 3:
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                robot_ctrl.update_and_publish(21)

            #出圆柱
            act(26)
            
            
        #高台
        if user_input<10:
            #朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,180):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue

            #居中
            while robot_ctrl.align(scan_node.get_scan()):
                continue

            #高台对齐
            while scan_node.get_scan()[1]>0.70:
                # print(scan_node.get_scan()[1])
                robot_ctrl.update_and_publish(39)
            
            #站立
            act(0)

            #爬高台
            act(22)

            #趴下
            act(24)

            #站立
            act(0)

            #角度补偿
            for _ in range(3):
                fix=robot_ctrl.rpy_fix_left(scan_node,y_xyz_node,180)
            fix_cos=math.cos(fix/180*math.pi)

            #朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,180):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue
                
            #居中
            while robot_ctrl.align(scan_node.get_scan()):
                continue

            #高台对齐
            while scan_node.get_scan()[1]>2.92:
                # print(scan_node.get_scan()[1])
                robot_ctrl.update_and_publish(39)
            

            
            #站立
            act(0)


            #爬高台
            act(22)


            #趴下
            act(24)



        # #下高台


            #站立
            act(0)

            #前进一小步
            act(29)

            #站立
            act(0)

            #跳跃
            act(25)

            
            #站立
            act(0)

            #后退
            act(40)
            #角度补偿
            for _ in range(5):
                fix=robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,180)
            fix_cos=math.cos(fix/180*math.pi)

            #朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,180):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue
            #居中
            while robot_ctrl.align(scan_node.get_scan()):
                continue


            #高台对齐
            while scan_node.get_scan()[1]>1.50:
                # print(scan_node.get_scan()[1])
                robot_ctrl.update_and_publish(39)


            #站立
            act(0)


            #跳跃
            act(25)


            #站立
            act(0)

            
        #直角转弯3
        if user_input<11:
            for _ in range(5):
                fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,180)
            fix_cos=math.cos(fix/180*math.pi)

            #行走至离墙面0.4m
            while scan_node.get_scan()[1] > 0.4:
                robot_ctrl.update_and_publish(7)


            #转弯
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,270):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue
           
        #上楼梯
        if user_input<12:
            while robot_ctrl.align2(scan_node.get_scan()):
                robot_ctrl.update_and_publish(7)
            #楼梯对齐
            act(36)

            while robot_ctrl.align_louti(scan_node.get_scan()):
                continue

            #楼梯对齐
            while scan_node.get_scan()[1]>0.76:
                # print(scan_node.get_scan()[1])
                robot_ctrl.update_and_publish(39)

            #站立
            act(0)

            #为独木桥开启tof
            head_tof_node=HeadTofSubscriber()
            executor.add_node(head_tof_node)

            #自定义上楼梯
            act(31)

            
            #站立
            act(0)

            
        #过独木桥
        if user_input<13:
            count = 0
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos > -2.2:
                # print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
                if count == 2:
                    act(33)

                    count=0
                if count == -2:
                    act(32)
                    count=0
                temp = head_tof_node.tof_dumuqiao()
                if temp == 1:
                    count+=1
                    robot_ctrl.update_and_publish(6)
                elif temp == -1:
                    count-=1
                    robot_ctrl.update_and_publish(5)
                else:
                    robot_ctrl.update_and_publish(7)
            #关闭tof
            executor.remove_node(head_tof_node)
            try:
                head_tof_node.destroy_node()
            except:
                print("head_tof_node has been destroyed")

        #下楼梯
        
        if user_input<14:
            while robot_ctrl.align2(scan_node.get_scan()):
                robot_ctrl.update_and_publish(34)
            
            act(41)


        #石板路
        if user_input<15:
            # 角度补偿
            for _ in range(5):
                fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,270)
            fix_cos=math.cos(fix/180*math.pi)

            #朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,270):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue
            #居中
            while robot_ctrl.align(scan_node.get_scan()):
                continue
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos > -1.5:
                # print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
                robot_ctrl.update_and_publish(35)

            #转弯
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,270):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue

            #居中
            while robot_ctrl.align(scan_node.get_scan()):
                continue

            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos > -1.3:
                # print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
                robot_ctrl.update_and_publish(35)

        #直角转弯4
        if user_input<16:
            #行走至离墙面0.2m
            while scan_node.get_scan()[1] > 0.4:
                robot_ctrl.update_and_publish(35)


            #转弯
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue

    #进入第四段
        #颜色识别，如果是偏向一边，可不可以，如果居中，可不可以。
        if user_input<17:
            #走到最后一段
            while robot_ctrl.align2(scan_node.get_scan()):
                robot_ctrl.update_and_publish(14)
            #角度补偿，很关键
            for _ in range(8):
                fix=robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
            fix_cos=math.cos(fix/180*math.pi)
            

            #根据腿式里程计前进50cm
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<0.7:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(7)
            #rpy朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue
            #居中对齐
            while robot_ctrl.align(scan_node.get_scan()):
                continue

            #开启realsense
            camera_thread = threading.Thread(target=start_camera)
            camera_thread.start()
            start_time = time.time()
            while time.time() - start_time < 25:
                robot_ctrl.update_and_publish(0)


        #第一块布
            # 创建并启动子进程
            thread1=threading.Thread(target=activate_camera)
            thread1.start()
            image_node1=ImageSubscriber()
            executor.add_node(image_node1)
            while image_node1.image is None:
                robot_ctrl.update_and_publish(0)
            thread11=threading.Thread(target=deactivate_camera)
            thread11.start()
            executor.remove_node(image_node1)
            flag=image_data_func(image_node1.image)
            try:
                image_node1.destroy_node()
            except:
                print("已清除image_node1")
            if flag:   #左移40cm
                xyz=y_xyz_node.xyz
                while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos<0.27:
                    # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                    robot_ctrl.update_and_publish(5)
            else:               #右移40cm
                xyz=y_xyz_node.xyz
                while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos>-0.27:
                    # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                    robot_ctrl.update_and_publish(6)

            #前进1m
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<0.88:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(7)
            
            # 角度补偿
            if flag:
                for _ in range(5):
                    fix = robot_ctrl.rpy_fix_left(scan_node,y_xyz_node,0)
                fix_cos=math.cos(fix/180*math.pi)
            else:
                for _ in range(5):
                    fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
                fix_cos=math.cos(fix/180*math.pi)

            #rpy朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue
            # 居中对齐
            while robot_ctrl.align(scan_node.get_scan()):
                continue



        #第二块布
            thread2=threading.Thread(target=activate_camera)
            thread2.start()
            image_node2=ImageSubscriber()
            executor.add_node(image_node2)
            while image_node2.image is None:
                robot_ctrl.update_and_publish(0)
            thread22=threading.Thread(target=deactivate_camera)
            thread22.start()
            executor.remove_node(image_node2)
            flag=image_data_func(image_node2.image)
            try:
                image_node2.destroy_node()
            except:
                print("已清除image_node1")
            if flag:   #左移40cm
                xyz=y_xyz_node.xyz
                while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos<0.27:
                    # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                    robot_ctrl.update_and_publish(5)
            else:               #右移40cm
                xyz=y_xyz_node.xyz
                while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos>-0.27:
                    # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                    robot_ctrl.update_and_publish(6)

            #前进1m
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<0.88:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(7)
            #rpy朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue
            #居中对齐
            while robot_ctrl.align(scan_node.get_scan()):
                continue




        #第三块布
            thread3=threading.Thread(target=activate_camera)
            thread3.start()
            image_node3=ImageSubscriber()
            executor.add_node(image_node3)
            while image_node3.image is None:
                robot_ctrl.update_and_publish(0)
            thread33=threading.Thread(target=deactivate_camera)
            thread33.start()
            executor.remove_node(image_node3)
            flag=image_data_func(image_node3.image)
            try:
                image_node3.destroy_node()
            except:
                print("已清除image_node1")
            if flag:   #左移40cm
                xyz=y_xyz_node.xyz
                while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos<0.27:
                    # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                    robot_ctrl.update_and_publish(5)
            else:               #右移40cm
                xyz=y_xyz_node.xyz
                while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos>-0.27:
                    # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                    robot_ctrl.update_and_publish(6)

            #前进1m
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<0.88:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(7)
            #rpy朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue
            #居中对齐
            while robot_ctrl.align(scan_node.get_scan()):
                continue




        #第四块布
            thread4=threading.Thread(target=activate_camera)
            thread4.start()
            image_node4=ImageSubscriber()
            executor.add_node(image_node4)
            while image_node4.image is None:
                robot_ctrl.update_and_publish(0)
            thread44=threading.Thread(target=deactivate_camera)
            thread44.start()
            executor.remove_node(image_node4)
            flag=image_data_func(image_node4.image)
            try:
                image_node4.destroy_node()
            except:
                print("已清除image_node1")
            if flag:   #左移40cm
                xyz=y_xyz_node.xyz
                while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos<0.27:
                    # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                    robot_ctrl.update_and_publish(5)
            else:               #右移40cm
                xyz=y_xyz_node.xyz
                while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos>-0.27:
                    # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                    robot_ctrl.update_and_publish(6)

            #前进1m
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<0.88:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(7)
            #rpy朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue
            #居中对齐
            while robot_ctrl.align(scan_node.get_scan()):
                continue


        #第五块布
            thread5=threading.Thread(target=activate_camera)
            thread5.start()
            image_node5=ImageSubscriber()
            executor.add_node(image_node5)
            while image_node5.image is None:
                robot_ctrl.update_and_publish(0)
            thread55=threading.Thread(target=deactivate_camera)
            thread55.start()
            executor.remove_node(image_node5)
            flag=image_data_func(image_node5.image)
            try:
                image_node5.destroy_node()
            except:
                print("已清除image_node1")
            if flag:   #左移40cm
                xyz=y_xyz_node.xyz
                while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos<0.27:
                    # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                    robot_ctrl.update_and_publish(5)
            else:               #右移40cm
                xyz=y_xyz_node.xyz
                while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos>-0.27:
                    # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                    robot_ctrl.update_and_publish(6)

            #前进1m
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<1:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(7)
            #rpy朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                continue
            
            #居中对齐
            while robot_ctrl.align(scan_node.get_scan()):
                continue
            # stop_camera()
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<1:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(7)
            while True:
                robot_ctrl.update_and_publish(0)
            


            




    except KeyboardInterrupt:
        robot_ctrl.update_and_publish("0")
        while lcm_status.rec_msg.order_process_bar<97:
            robot_ctrl.update_and_publish("0")
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()