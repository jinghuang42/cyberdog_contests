import rclpy
import time
import lcm
import math
import sys
import toml
import os
from threading import Thread
from collections import deque
from rclpy.executors import MultiThreadedExecutor

from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from lcm_status import LcmStatus
from y_xyz import LCMListener
from scan import ROSListener

class CyberdogRaceMaster:
    def __init__(self, init_angle=0):
        self.lidar = ROSListener()
        self.odom = LCMListener()
        self.status = LcmStatus()
        self.ctrl_lc = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.msg = robot_control_cmd_lcmt()
        
        # --- 核心控制变量 ---
        self.init_angle = init_angle
        self.stage = 0
        self.sub_stage = 0
        self.task_id = 0      # 0-直角弯前, 1-过渡, 2-圆柱前
        self.life_count = 0
        self.is_correcting = False 
        self.turn_done_xyz = [0.0, 0.0, 0.0] 
        self.start_xyz = None # 记录比赛起点
        self.last_log_time = 0
        
        # --- 优化项：滤波与保护 ---
        self.f_dist_filter = deque(maxlen=5) # 雷达前向滤波
        self.total_dist = 0.0

        # 加载步态文件
        self.load_files()
        self.status.run()

    def load_files(self):
        """ 智能路径寻找 """
        base = os.path.dirname(os.path.abspath(__file__))
        try:
            # 基础步态
            self.steps = toml.load(os.path.join(base, "../../toml/usergait.toml"))
            # 高台步态
            self.gaotai_steps = toml.load(os.path.join(base, "../../gaotai.toml"))["step"]
            print(">>> [系统] 步态文件加载成功")
        except: 
            print(">>> [错误] 路径识别失败，请检查TOML位置")

    def publish(self, gait_idx, v_x=None, v_y=0.0, v_yaw=None, h=0.08, p_z=0.03):
        """ 保持原 publish 逻辑，仅补齐数组长度防止崩溃 """
        try:
            step = self.steps["step"][int(gait_idx)]
            self.msg.mode, self.msg.gait_id = step["mode"], step["gait_id"]
            self.msg.contact, self.msg.value, self.msg.duration = step["contact"], step["value"], step["duration"]
            vx = v_x if v_x is not None else step["vel_des"][0]
            vyaw = v_yaw if v_yaw is not None else step["vel_des"][2]
            self.msg.vel_des = [vx, v_y, vyaw]
            self.msg.step_height, self.msg.pos_des = [h, h], [0.0, 0.0, p_z]
            
            # --- 修复：LCM 数组补齐 ---
            self.msg.acc_des = [0.0] * 6
            for i in range(3):
                self.msg.rpy_des[i] = step["rpy_des"][i]
                self.msg.foot_pose[i] = step["foot_pose"][i]
                self.msg.ctrl_point[i] = step["ctrl_point"][i]
                
            self.msg.life_count = self.life_count % 128
            self.life_count += 1
            self.ctrl_lc.publish("robot_control_cmd", self.msg.encode())
        except: pass

    def run_competition(self):
        print(">>> 启动：圆柱逻辑回退版 (仅优化起步左转)")
        while True:
            scan = self.lidar.get_scan()
            curr_yaw = self.odom.y
            curr_xyz = self.odom.xyz
            
            if scan is None or curr_yaw is None or curr_xyz is None:
                time.sleep(0.01); continue
            
            # 记录起点
            if self.start_xyz is None: self.start_xyz = curr_xyz
            
            L, F, R, LF, RF = scan
            
            # --- 关键：滤波与位移计算 ---
            self.f_dist_filter.append(F)
            f_smooth = sum(self.f_dist_filter) / len(self.f_dist_filter)
            self.total_dist = math.sqrt((curr_xyz[0]-self.start_xyz[0])**2 + (curr_xyz[1]-self.start_xyz[1])**2)
            dist_from_turn = math.sqrt((curr_xyz[0]-self.turn_done_xyz[0])**2 + (curr_xyz[1]-self.turn_done_xyz[1])**2)

            # --- 状态机逻辑 (保持原版不变) ---

            if self.stage == 0: # 起立
                self.publish(0)
                if self.status.rec_msg.order_process_bar > 0: self.stage = 1

            elif self.stage == 1: # 巡航与识别
                if self.task_id == 0: # 寻找直角弯
                    # 【核心修改点】：增加 0.8米位移锁 和 雷达滤波判定
                    if self.total_dist > 0.8 and f_smooth < 0.23: 
                        print(f">>> [直角弯] 触发！位移: {self.total_dist:.2f}m")
                        self.stage = 2
                
                elif self.task_id == 1: # 转弯后安全过渡
                    if dist_from_turn > 0.4: self.task_id = 2

                elif self.task_id == 2: # 寻找圆柱
                    if F < 0.3 and (LF < 1.0 or L < 1.0):
                        print(">>> [圆柱] 触发避障")
                        self.stage = 3; self.sub_stage = 1
                        self.sub_stage_start_time = time.time()

                # --- 居中对齐逻辑 ---
                vx_target = 0.20 if F > 0.6 else 0.12
                diff = L - R
                if self.is_correcting:
                    self.publish(5 if diff > 0 else 6, v_x=0.15, v_y=(0.06 if diff > 0 else -0.06))
                    if abs(diff) < 0.08: self.is_correcting = False
                else:
                    self.publish(7, v_x=vx_target)
                    if abs(diff) > 0.18: self.is_correcting = True

            elif self.stage == 2: # 执行直角左转
                target_yaw = (curr_yaw + 90) % 360
                while True:
                    error = (target_yaw - self.odom.y + 180) % 360 - 180
                    if abs(error) < 4: break
                    self.publish(2, v_yaw=0.5)
                    time.sleep(0.05)
                self.init_angle = target_yaw
                self.turn_done_xyz = list(self.odom.xyz)
                self.task_id = 1; self.stage = 1

            elif self.stage == 3: # 圆柱避障核心 (完全维持原逻辑)
                yaw_err = (self.init_angle - curr_yaw + 180) % 360 - 180
                if self.sub_stage == 1:
                    self.publish(7, v_x=0.1, v_y=-0.45, v_yaw=-0.55) 
                    if time.time() - self.sub_stage_start_time > 2.2: self.sub_stage = 2
                elif self.sub_stage == 2:
                    target_l = 0.65
                    v_yaw_cmd = 0.70 + (L - target_l) * 1.5
                    self.publish(7, v_x=0.22, v_yaw=v_yaw_cmd)
                    if RF > 1.8 or R > 1.8:
                        self.sub_stage = 3; self.sub_stage_start_time = time.time()
                elif self.sub_stage == 3:
                    if abs(yaw_err) > 8: self.publish(7, v_x=0.12, v_yaw=-0.6)
                    else:
                        if time.time() - self.sub_stage_start_time < 1.5: self.publish(7, v_x=0.20, v_yaw=0.0)
                        else:
                            self.last_pillar_time = time.time()
                            self.stage = 5; self.sub_stage = 0

            elif self.stage == 5: # 脱离阶段
                if time.time() - self.last_pillar_time < 3.0:
                    diff = L - R
                    self.publish(7, v_x=0.25, v_y=(0.06 if diff > 0 else -0.06) if abs(diff)>0.1 else 0.0)
                else:
                    self.task_id = 0; self.stage = 1

            time.sleep(0.05)

def main():
    rclpy.init()
    master = CyberdogRaceMaster(init_angle=0)
    executor = MultiThreadedExecutor()
    executor.add_node(master.lidar)
    Thread(target=executor.spin, daemon=True).start()
    master.run_competition()

if __name__ == '__main__':
    main()