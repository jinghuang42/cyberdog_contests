import toml
import time
import lcm
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from monitoring2 import LcmStatus
from head_tof_func import get_head_tof_node
#发送指令的类，这个写成按照需要发送指令，如果一直发送会存在问题。
class Robot_Ctrl(object):
    def __init__(self):
        self.lc = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.msg = robot_control_cmd_lcmt()
        self.steps = toml.load("./tomls/usergait.toml")
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
                self.update_and_publish(3)
            else:
                self.update_and_publish(2)
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
        print(abs(scan_data[0]-scan_data[2]))
        if abs(scan_data[0]-scan_data[2])<0.02:
            return False
        else:
            print("对齐中")
            if scan_data[0]>scan_data[2]:
                self.update_and_publish(5)
            else:
                self.update_and_publish(6)
            return True


def main():
    #存储腿式里程计的信息
    robot_ctrl = Robot_Ctrl()
    lcm_status = LcmStatus()
    tof_node=get_head_tof_node()
    lcm_status.run()
    #这种情况下根据上一个指令执行是否完成下发下一个指令。这个心跳包是什么情况，如果狗狗一直在运动，是不是就是不需要心跳包。不是的，狗子在运行过程中需要一直发送心跳包
    #这种情况下是不是应该开一个线程执行指令，还是在原来的线程里面执行指令，如果在原来的线程上执行指令，是否会阻塞，这里是否可以阻塞。可以在原来的线程中继续执行，可以阻塞。
    #存在一个问题，如果在帘子那一关执行判断需要花比较长的时间，心跳包是否还会延续，如果不能延续只能开一个线程执行。
    try:
        robot_ctrl.update_and_publish(0)
        while lcm_status.rec_msg.order_process_bar==100:
            robot_ctrl.update_and_publish(0)
        while lcm_status.rec_msg.order_process_bar<100:
            robot_ctrl.update_and_publish(0)
        while tof_node.tof_gaotai():
            robot_ctrl.update_and_publish(1)
        while True:
            robot_ctrl.update_and_publish(0)
    except KeyboardInterrupt:
        robot_ctrl.update_and_publish("趴下")
        while lcm_status.rec_msg.order_process_bar<97:
            robot_ctrl.update_and_publish("趴下")


if __name__ == '__main__':
    main()