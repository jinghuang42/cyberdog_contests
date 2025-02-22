import time
import lcm
import math
from threading import Thread
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from localization_lcmt import localization_lcmt

class PubMsg(object):
    def __init__(self):
        self.pub_thread = Thread(target=self.pub_response)
        self.rec_thread = Thread(target=self.rec_response)
        self.input_thread = Thread(target=self.get_user_input)
        self.lc_p = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.running = False
        self.rec_msg = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
        self.y = 0.0
        self.usr_num = None
        self.life_count=0

    def run(self):
        self.running = True
        self.pub_thread.start()
        self.rec_msg.subscribe("global_to_robot", self.msg_handler)
        self.rec_thread.start()
        self.input_thread.start()

    def get_user_input(self):
        while self.running:
            try:
                user_input = input("请输入目标角度: ")
                self.usr_num = float(user_input)
            except ValueError:
                print("请输入有效的数字")

    def pub_response(self):
        while self.running:
            if self.usr_num is not None and abs(self.y - self.usr_num) > 2.0:
                msg = robot_control_cmd_lcmt()
                msg.mode = 11
                msg.gait_id = 27
                msg.contact = 15
                self.life_count+=1
                msg.life_count = self.life_count%128
                msg.vel_des = [0.0, 0.0, 0.3]
                msg.rpy_des = [0.0, 0.0, 0.0]
                msg.pos_des = [0.0, 0.0, 0.0]
                msg.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                msg.ctrl_point = [0.0, 0.0, 0.0]
                msg.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                msg.step_height = [0.12, 0.12]
                msg.value = 0
                msg.duration = 0
                self.lc_p.publish("robot_control_cmd", msg.encode())
            else:
                msg = robot_control_cmd_lcmt()
                msg.mode = 12
                msg.gait_id = 0
                msg.contact = 0
                self.life_count+=1
                msg.life_count = self.life_count%128
                msg.vel_des = [0.0, 0.0, 0.0]
                msg.rpy_des = [0.0, 0.0, 0.0]
                msg.pos_des = [0.0, 0.0, 0.0]
                msg.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                msg.ctrl_point = [0.0, 0.0, 0.0]
                msg.foot_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                msg.step_height = [0.0, 0.0]
                msg.value = 0
                msg.duration = 5000
                self.lc_p.publish("robot_control_cmd", msg.encode())

    def msg_handler(self, channel, data):
        rec_msg = localization_lcmt().decode(data)
        rec_msg.rpy = list(rec_msg.rpy)
        for i in range(len(rec_msg.rpy)):
            rec_msg.rpy[i] = rec_msg.rpy[i] * 180 / math.pi
        flag = True if abs((abs(rec_msg.rpy[0]) - 0)) < abs((abs(rec_msg.rpy[0]) - 180)) else False
        rpy = rec_msg.rpy[:]
        if not flag:
            rpy[2] = rpy[2] + 180
        self.y = rpy[2]

    def rec_response(self):
        while self.running:
            try:
                self.rec_msg.handle()
            except:
                print("LCM handle exception occurred")

    def quit(self):
        self.running = False
        self.pub_thread.join()
        self.rec_thread.join()
        self.input_thread.join()

pub_msg = PubMsg()
pub_msg.run()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pub_msg.quit()