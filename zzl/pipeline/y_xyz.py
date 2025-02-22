import lcm
from localization_lcmt import localization_lcmt
from threading import Thread
import math
import time

class LCMListener:
    def __init__(self):
        self.y = None
        self.xyz = None
        self.lcm_rec = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
        self.lcm_rec.subscribe("global_to_robot", self.lcm_handler)
        
        # 创建 LCM 线程
        self.lcm_thread = Thread(target=self.lcm_handle)
        self.lcm_thread.start()

    def lcm_handler(self, channel, data):
        msg = localization_lcmt().decode(data)
        msg.rpy = list(msg.rpy)
        for i in range(len(msg.rpy)):
            msg.rpy[i] = msg.rpy[i] * 180 / math.pi
        flag = True if abs(abs(msg.rpy[0]) - 0) < abs(abs(msg.rpy[0]) - 180) else False
        if not flag:
            self.y = msg.rpy[2] + 180
        else:
            self.y = msg.rpy[2]
        self.y = self.y + 360 if self.y < 0 else self.y
        
        self.y = self.y % 360
        self.xyz = msg.xyz

    def lcm_handle(self):
        while True:
            self.lcm_rec.handle()
if __name__ == '__main__':
    lcm_listener = LCMListener()
    while True:
        print(lcm_listener.y, lcm_listener.xyz)
