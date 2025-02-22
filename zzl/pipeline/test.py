from scan import start_scan_listener
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
from get_pic_func import ImageProcessor
import numpy as np

fix = 0
fix_cos=1

#发送指令的类，这个写成按照需要发送指令，如果一直发送会存在问题。
class Robot_Ctrl(object):
    def __init__(self):
        self.lc = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.msg = robot_control_cmd_lcmt()
        self.steps = toml.load("./toml/usergait.toml")
        self.life_count = 0
        self.last_execution_time = 0
    def update_and_publish(self,num):
        current_time = time.time()
        if current_time - self.last_execution_time < 0.04:
            return
        self.last_execution_time=current_time
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
        time.sleep(0.25)
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
        if abs(scan_data[0]-scan_data[2])<0.02:
            return False
        else:
            if scan_data[0]>scan_data[2]:
                self.update_and_publish(5)
            else:
                self.update_and_publish(6)
            return True
    def align2(self,scan_data):
        return scan_data[0]+scan_data[2]>1.4
        #scan_node.scan     0.905
    def rpy_fix_right(self,scan_node,cur_angle,tar_angle):
        if scan_node.scan[2] and scan_node.scan[4]!=float('inf'):
            while scan_node.scan[2]/scan_node.scan[4] < 0.907 or scan_node.scan[2]/scan_node.scan[4] >0.91:
                if scan_node.scan[2]/scan_node.scan[4] > 0.91:
                    self.update_and_publish(27)
                elif scan_node.scan[2]/scan_node.scan[4] < 0.907:
                    self.update_and_publish(28)
            return tar_angle-cur_angle.y
        else:
            while scan_node.scan[0]/scan_node.scan[3] < 0.907 or scan_node.scan[0]/scan_node.scan[3] > 0.91:
                if scan_node.scan[0]/scan_node.scan[3] > 0.91:
                    self.update_and_publish(27)
                elif scan_node.scan[0]/scan_node.scan[3] <0.907:
                    self.update_and_publish(28)
            return tar_angle-cur_angle.y
    def image_left_right(self,image_data):
        # 提取 R 和 G 通道
        # r_channel = image_data[:, :, 2]
        g_channel = image_data[:, :, 1]

        # 分割图像为左半部和右半部
        # left_r_half = r_channel[:, :320]
        # right_r_half = r_channel[:, 320:]
        left_g_half = g_channel[:, :320]
        right_g_half = g_channel[:, 320:]

        # 计算每半部分的 R 通道和 G 通道平均值
        # self.left_r_mean = np.mean(left_r_half)
        # self.right_r_mean = np.mean(right_r_half)
        self.left_g_mean = np.mean(left_g_half)
        self.right_g_mean = np.mean(right_g_half)
        return self.left_g_mean>self.right_g_mean

def main():
    if len(sys.argv) < 2:
        return
    user_input = int(sys.argv[1])  # 接收命令行参数
    #存储腿式里程计的信息
    xyz=None
    rclpy.init(args=None)
    scan_node = ROSListener()
    head_tof_node = HeadTofSubscriber()
    image_node=ImageProcessor()
    executor=MultiThreadedExecutor()
    executor.add_node(head_tof_node)
    executor.add_node(scan_node)
    executor.add_node(image_node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    y_xyz_node = LCMListener()
    robot_ctrl = Robot_Ctrl()
    lcm_status = LcmStatus()
    lcm_status.run()
    #这种情况下根据上一个指令执行是否完成下发下一个指令。这个心跳包是什么情况，如果狗狗一直在运动，是不是就是不需要心跳包。不是的，狗子在运行过程中需要一直发送心跳包
    #这种情况下是不是应该开一个线程执行指令，还是在原来的线程里面执行指令，如果在原来的线程上执行指令，是否会阻塞，这里是否可以阻塞。可以在原来的线程中继续执行，可以阻塞。
    #存在一个问题，如果在帘子那一关执行判断需要花比较长的时间，心跳包是否还会延续，如果不能延续只能开一个线程执行。
    try:
        #1.起始蹲下，执行站立
        while lcm_status.rec_msg.order_process_bar==100:
            robot_ctrl.update_and_publish(0)
        while lcm_status.rec_msg.order_process_bar<100:
            robot_ctrl.update_and_publish(0)

        
        #圆柱
        if user_input<9:
            #角度补偿
            for _ in range(3):
                fix=robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,180)
            fix_cos=math.cos(fix/180*math.pi)
            #朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,180):
                continue
            #居中
            while robot_ctrl.align(scan_node.scan):
                continue

            #前进至圆柱前0.2m
            while scan_node.scan[1] > 0.20:
                robot_ctrl.update_and_publish(7)
            
            #旋转
            while abs((y_xyz_node.y+fix+360)%360-90) > 3:
                robot_ctrl.update_and_publish(20)

            #后退
            xyz=y_xyz_node.xyz
            
            while (y_xyz_node.xyz[1] - xyz[1])/fix_cos > -0.20:
                robot_ctrl.update_and_publish(18)

            #前进至圆柱前0.2m
            while scan_node.scan[0] > 0.3:
                robot_ctrl.update_and_publish(5)

            #绕圆台
            while abs((y_xyz_node.y+fix+360)%360-230) > 3:
                robot_ctrl.update_and_publish(21)

            #出圆柱
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(26)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(26)
            
            
        #高台
        if user_input<10:
            #朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,180):
                continue

            #居中
            while robot_ctrl.align(scan_node.scan):
                continue

            #高台对齐
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(17)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(17)
            
            #站立
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(0)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(0)

            #爬高台
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(22)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(22)

            #趴下
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(24)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(24)

            #站立
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(0)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(0)


            #高台对齐
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(23)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(23)
            
            #站立
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(0)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(0)

            #爬高台
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(22)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(22)

            #趴下
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(24)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(24)

            #站立
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(0)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(0)

        # #下高台
            #走一小步
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(29)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(29)

            #站立
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(0)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(0)

            #跳跃
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(25)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(25)
            
            #站立
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(0)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(0)

            #走一小步
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(30)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(30)

            #站立
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(0)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(0)

            #跳跃
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(25)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(25)

            #站立
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(0)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(0)

            
        #直角转弯3
        if user_input<11:
            for _ in range(3):
                fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,180)
            fix_cos=math.cos(fix/180*math.pi)

            #行走至离墙面0.2m
            while scan_node.scan[1] > 0.2:
                robot_ctrl.update_and_publish(7)

            #后退至离墙面0.4m
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos > -0.3:
                robot_ctrl.update_and_publish(18)

            #转弯
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,270):
                continue

        #上楼梯
        if user_input<12:
            while robot_ctrl.align2(scan_node.scan):
                robot_ctrl.update_and_publish(7)
            #楼梯对齐
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(23)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(23)
            while robot_ctrl.align(scan_node.scan):
                continue
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(23)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(23)
            #站立
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(0)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(0)

            #自定义上楼梯
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(31)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(31)
            
            #站立
            while lcm_status.rec_msg.order_process_bar==100:
                robot_ctrl.update_and_publish(0)
            while lcm_status.rec_msg.order_process_bar<100:
                robot_ctrl.update_and_publish(0)
            
        #过独木桥
        if user_input<13:
            count = 0
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos > -2.2:
                if count == 2:
                    while lcm_status.rec_msg.order_process_bar==100:
                        robot_ctrl.update_and_publish(33)
                    while lcm_status.rec_msg.order_process_bar<100:
                        robot_ctrl.update_and_publish(33) 
                    count=0
                if count == -2:
                    while lcm_status.rec_msg.order_process_bar==100:
                        robot_ctrl.update_and_publish(32)
                    while lcm_status.rec_msg.order_process_bar<100:
                        robot_ctrl.update_and_publish(32) 
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

        #下楼梯
        if user_input<14:
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos > -1.8:
                robot_ctrl.update_and_publish("下楼梯")


        #石板路
        if user_input<15:
            #居中
            while robot_ctrl.align(scan_node.scan):
                continue
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos > -2.5:
                robot_ctrl.update_and_publish("石板路")
        #直角转弯4
        if user_input<16:
            #行走至离墙面0.2m
            while scan_node.scan[1] > 0.2:
                robot_ctrl.update_and_publish(7)

            #后退至离墙面0.4m
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos < 0.2:
                robot_ctrl.update_and_publish(18)

            #转弯
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                continue

    #进入第四段
        #颜色识别
        if user_input<17:
            image_data=image_node.image
            if robot_ctrl.image_left_right(image_data):
                while True:
                    robot_ctrl.update_and_publish(0)
            while True:
                robot_ctrl.update_and_publish(24)
            



            #走到最后一段
            while robot_ctrl.align2(scan_node.scan):
                robot_ctrl.update_and_publish(14)
            for _ in range(3):
                fix=robot_ctrl.rpy_fix_right()
            fix_cos=math.cos(fix/180*math.pi)
            
            #根据腿式里程计前进50cm
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<0.5:
                robot_ctrl.update_and_publish(7)
            #rpy朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                continue
            #居中对齐
            while robot_ctrl.align(scan_node.scan):
                continue
            #左前1m
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<1:
                robot_ctrl.update_and_publish(12)
            #rpy朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                continue
            #居中对齐
            while robot_ctrl.align(scan_node.scan):
                continue
            #右前1m
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<1:
                robot_ctrl.update_and_publish(13)
            #rpy朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                continue
            #居中对齐
            while robot_ctrl.align(scan_node.scan):
                continue
            #左前1m
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<1:
                robot_ctrl.update_and_publish(12)
            #rpy朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                continue
            #居中对齐
            while robot_ctrl.align(scan_node.scan):
                continue
            #左前1m
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<1:
                robot_ctrl.update_and_publish(12)
            #rpy朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                continue
            #居中对齐
            while robot_ctrl.align(scan_node.scan):
                continue
            #右前1m
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<1:
                robot_ctrl.update_and_publish(13)
            #rpy朝前
            while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
                continue
            #居中对齐
            while robot_ctrl.align(scan_node.scan):
                continue
            while True:
                robot_ctrl.update_and_publish(0)
        if user_input>30:
            while True:
                robot_ctrl.update_and_publish(0)

    except KeyboardInterrupt:
        robot_ctrl.update_and_publish("0")
        while lcm_status.rec_msg.order_process_bar<97:
            robot_ctrl.update_and_publish("0")
    finally:
        executor.shutdown()
        scan_node.destory_node()
        head_tof_node.destory_node()
        image_node.destory_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()