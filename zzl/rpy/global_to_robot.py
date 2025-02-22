import sys
import time
import lcm
import math
from threading import Thread, Lock
# from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from localization_lcmt import localization_lcmt

class Rec_msg(object):
    def __init__(self):
        self.rec_thread = Thread(target=self.rec_responce)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
        self.rec_msg = localization_lcmt()
        self.running = False

    def run(self):
        self.running = True
        self.lc_r.subscribe("global_to_robot", self.msg_handler)
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        self.rec_msg = localization_lcmt().decode(data)
        self.rec_msg.rpy=list(self.rec_msg.rpy)
        for i in range(len(self.rec_msg.rpy)):
            self.rec_msg.rpy[i]=self.rec_msg.rpy[i]*180/math.pi
        flag=True if abs((abs(self.rec_msg.rpy[0])-0))<abs((abs(self.rec_msg.rpy[0])-180)) else False
        self.rpy=self.rec_msg.rpy[:]
        if (flag==False):
            self.rpy[2]=self.rpy[2]+180
        print(f"机身位置{self.rec_msg.xyz}\n 机身姿态{self.rpy[2]}")

    def rec_responce(self): #每隔多少时间打印信息
        while self.running:
            try:
                self.lc_r.handle()
            except:
                print("LCM handle exception occurred")

    def quit(self):
        self.running = False
        self.rec_thread.join()

rec_msg = Rec_msg()
rec_msg.run()