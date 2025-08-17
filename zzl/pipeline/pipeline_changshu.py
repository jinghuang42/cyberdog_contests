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
    def act(x):
        while lcm_status.rec_msg.order_process_bar==100:
            robot_ctrl.update_and_publish(x)
        while lcm_status.rec_msg.order_process_bar<100:
            robot_ctrl.update_and_publish(x)


    try:
        #1.起始蹲下，执行站立
        print("开始站立")
        act(0)
        print("站立完成")

        print('通过独木桥')
        while robot_ctrl.align2(scan_node.get_scan()):
            robot_ctrl.update_and_publish(7)
        for _ in range(5):
            fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
        fix_cos=math.cos(fix/180*math.pi)  
        xyz=y_xyz_node.xyz
        while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos < 0.3:
            # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
            robot_ctrl.update_and_publish(42)

        while robot_ctrl.align(scan_node.get_scan()):
            continue

        for _ in range(5):
            fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
        fix_cos=math.cos(fix/180*math.pi)  
        xyz=y_xyz_node.xyz
        while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos < 0.5:
            # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
            robot_ctrl.update_and_publish(42)

        while robot_ctrl.align(scan_node.get_scan()):
            continue

    #直角转弯2
        print("开始直角转弯")
        while scan_node.get_scan()[1] > 0.4:
            robot_ctrl.update_and_publish(42)
         #转弯
        while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
            # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
            continue
        print("直角转弯结束")

#         # 2.通过幕布




        print('前进并剧中')
        while robot_ctrl.align2(scan_node.get_scan()):
            robot_ctrl.update_and_publish(7)
        for _ in range(5):
            fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
        fix_cos=math.cos(fix/180*math.pi)  
        xyz=y_xyz_node.xyz
        while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos < 0.2:
            # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
            robot_ctrl.update_and_publish(42)
        #居中
        print('通过第一块幕布')
        while robot_ctrl.align(scan_node.get_scan(),0.5):
            continue

        
        for _ in range(5):
            fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
        fix_cos=math.cos(fix/180*math.pi)  
        xyz=y_xyz_node.xyz
        while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos < 1.1:
            # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
            robot_ctrl.update_and_publish(7)


        print('通过第二块幕布')
        #居中
        while robot_ctrl.align(scan_node.get_scan(),-0.5):
            continue
        for _ in range(5):
            fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
        fix_cos=math.cos(fix/180*math.pi)  
        xyz=y_xyz_node.xyz
        while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos < 0.9:
            # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
            robot_ctrl.update_and_publish(7)

        print('通过第三块幕布')
        #居中
        while robot_ctrl.align(scan_node.get_scan(),0.5):
            continue
        for _ in range(5):
            fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
        fix_cos=math.cos(fix/180*math.pi)  
        xyz=y_xyz_node.xyz
        while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos < 0.9:
            # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
            robot_ctrl.update_and_publish(7)

    #直角转弯2
        print("开始直角转弯")


        while not robot_ctrl.align2(scan_node.get_scan()):
            robot_ctrl.update_and_publish(7)        
        while scan_node.get_scan()[1] > 0.4:
            robot_ctrl.update_and_publish(7)
         #转弯
        while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
            # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
            continue
        print("直角转弯结束")



        print('前进并剧中')
        while robot_ctrl.align2(scan_node.get_scan()):
            robot_ctrl.update_and_publish(7)

        #居中

        while robot_ctrl.align(scan_node.get_scan()):
            continue        



# #     #     # 睡5s
#     #     #time.sleep(5)


        #直行
        print("开始直立行走")
        #角度补偿
        for _ in range(5):
            fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
        fix_cos=math.cos(fix/180*math.pi)  
        print('角度补偿完成')
        #2.前进0.5m，根据腿式里程计
        xyz=y_xyz_node.xyz
        while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos < 0.5:
            #  print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
            robot_ctrl.update_and_publish(7)
        print("行走结束，即将进入沙地")

    #沙地
        print("开始沙地行走")
        #根据雷达判断进入第一阶段赛段
        while robot_ctrl.align2(scan_node.get_scan()):
            robot_ctrl.update_and_publish(7)
        print("居中对齐")
        #居中对齐
        while robot_ctrl.align(scan_node.get_scan()):
            continue
        print("居中对齐结束")
        #前进1.5m通过沙地
        xyz=y_xyz_node.xyz
        while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos < 1.524:
            print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
            robot_ctrl.update_and_publish(7)
        print("沙地结束，即将进入石子路")


    #石子路
            #朝前
        print("开始石子路")
        #while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
        #    continue
        print("开始居中对齐")
        #居中对齐
        while robot_ctrl.align(scan_node.get_scan()):
            continue
        print("居中对齐结束")
        #6前进，根据腿式里程计判断，前进1.5米通过石子路
        xyz=y_xyz_node.xyz
        while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos < 1.824:
            # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
            robot_ctrl.update_and_publish(19)
        print("石子路结束，即将上斜坡")
            
    #上斜坡
            #上0.3m对齐
        print("开始上斜坡")
        xyz=y_xyz_node.xyz
        while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos < 0.9:
            # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
            robot_ctrl.update_and_publish(7)

        #在斜坡上朝前
        #while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
        #    # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
        #    continue
        print("上斜坡结束，即将达到斜坡顶端")
        print("开始居中对齐")
        #居中
        #while robot_ctrl.align(scan_node.get_scan()):
        #    continue
        print("居中对齐结束")

    # 斜坡顶端
        #6前进，根据腿式里程计判断，前进0.7米到斜坡顶端
        print("斜坡顶端")
        xyz=y_xyz_node.xyz
        while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos < 0.75:
            # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
            robot_ctrl.update_and_publish(7)
        print("斜坡顶端结束")
       
    #下斜坡   
        # if user_input<6:
            #下1.4m对齐
        print("开始下斜坡")
        xyz=y_xyz_node.xyz
        while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos < 0.952:
            # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
            robot_ctrl.update_and_publish(7)

        #rpy超前
       # while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
            # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
       #     continue

        #居中
        while robot_ctrl.align2(scan_node.get_scan()):
            robot_ctrl.update_and_publish(7)
        while robot_ctrl.align(scan_node.get_scan()):
            continue
        print("下斜坡结束")

        #根据雷达前进至第二赛道
        print("开始减速带")
        while robot_ctrl.align2(scan_node.get_scan()):
            robot_ctrl.update_and_publish(7)

        act(23)
        #角度补偿
        for _ in range(3):
            fix=robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,180)
        fix_cos=math.cos(fix/180*math.pi)
        #朝前
        #while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,90):
            # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
         #   continue
        #居中
        while robot_ctrl.align(scan_node.get_scan()):
            continue
        #前进2.8通过减速带
        xyz=y_xyz_node.xyz
        while y_xyz_node.xyz[0]/fix_cos - xyz[0]/fix_cos > 0.8:
            # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
            robot_ctrl.update_and_publish(19)
        print("减速带结束")

    #直角转弯1

        print("JSK:开始直角转弯")
        while scan_node.get_scan()[1] > 0.4:
            robot_ctrl.update_and_publish(7)

        #转弯
        while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,270):
            # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
            continue
        print("直角转弯结束")
            
    #圆柱   
        #朝前
        print("JSK:调整位置，靠近圆柱")
        # xyz=y_xyz_node.xyz
        # while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos < 1.2:
        #     #  print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
        #     robot_ctrl.update_and_publish(7)

        print("柱子前0.3米")
        # 前进至圆柱前0.2m
        while scan_node.get_scan()[1] > 0.2:
            robot_ctrl.update_and_publish(7)
        #居中
        while robot_ctrl.align(scan_node.get_scan(),mv_data=-0.35):
            continue      

            
    #直角转弯2
        print("开始直角转弯")
        while scan_node.get_scan()[1] > 0.4:
            robot_ctrl.update_and_publish(7)
         #转弯
        while robot_ctrl.turn_left_or_right((y_xyz_node.y+fix+360)%360,0):
            print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
            continue
        print("直角转弯结束")


        act(0)

    except KeyboardInterrupt:
        robot_ctrl.update_and_publish("0")
        while lcm_status.rec_msg.order_process_bar<97:
            robot_ctrl.update_and_publish("0")
    finally:
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
