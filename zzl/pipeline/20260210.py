import rclpy
import time
import lcm
import math
import sys
import toml
from threading import Thread
from rclpy.executors import MultiThreadedExecutor

# 导入用户模块
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from lcm_status import LcmStatus
from y_xyz import LCMListener
from scan import ROSListener

class CyberdogRaceMaster:
    def __init__(self, init_angle=0):
        # 1. 传感器初始化
        self.lidar = ROSListener()
        self.odom = LCMListener()
        self.status = LcmStatus()
        self.ctrl_lc = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.msg = robot_control_cmd_lcmt()
        
        # 2. 状态控制
        self.init_angle = init_angle 
        self.stage = 0        
        self.sub_stage = 0    
        self.task_id = 0 # 0:弯1, 1:过渡, 2:圆柱, 3:弯2, 4:高台前对齐
        self.life_count = 0
        self.is_correcting = False 
        self.turn_done_xyz = [0.0, 0.0, 0.0] 
        self.sub_stage_start_time = 0 
        self.last_log_time = 0

        try:
            self.steps = toml.load("./toml/usergait.toml")
            print(">>> [系统] 满血完整版加载成功。")
        except Exception as e:
            print(f">>> [错误] 加载失败: {e}"); sys.exit(1)

        self.status.run()

    # ==================== 基础函数：完全保持你第一版的样子 ====================
    def publish(self, gait_idx, v_x=None, v_y=None, v_yaw=None, h=0.08, p_z=0.035):
        """修复了 LCM 数组打包长度的发布函数，确保不趴下"""
        try:
            step = self.steps["step"][int(gait_idx)]
            self.msg.mode, self.msg.gait_id = step["mode"], step["gait_id"]
            self.msg.contact, self.msg.value, self.msg.duration = step["contact"], step["value"], step["duration"]
            vx = v_x if v_x is not None else step["vel_des"][0]
            vy = v_y if v_y is not None else step["vel_des"][1]
            vyaw = v_yaw if v_yaw is not None else step["vel_des"][2]
            self.msg.vel_des = [vx, vy, vyaw]
            self.msg.step_height, self.msg.pos_des = [h, h], [0.0, 0.0, p_z]
            
            # --- 关键：补齐 acc_des 和 foot_pose 数组长度至 6 位 ---
            self.msg.acc_des = [0.0] * 6
            self.msg.foot_pose = [0.0] * 6
            self.msg.ctrl_point = [0.0] * 6
            for i in range(3):
                self.msg.rpy_des[i] = step["rpy_des"][i]
                self.msg.foot_pose[i] = step["foot_pose"][i]
                self.msg.ctrl_point[i] = step["ctrl_point"][i]
                
            self.msg.life_count = self.life_count % 128
            self.life_count += 1
            self.ctrl_lc.publish("robot_control_cmd", self.msg.encode())
        except: pass

    def act(self, gait_idx, custom_h=0.08, custom_pz=0.035):
        """执行 Static Gait 动作序列 (如动作22)"""
        print(f">>> [动作] 执行 Gait {gait_idx}")
        time.sleep(0.1)
        while self.status.rec_msg.order_process_bar >= 100:
            self.publish(gait_idx, h=custom_h, p_z=custom_pz); time.sleep(0.02)
        while self.status.rec_msg.order_process_bar < 98:
            self.publish(gait_idx, h=custom_h, p_z=custom_pz); time.sleep(0.02)

    # ==================== 新增高台辅助函数：严格匹配你要求的逻辑 ====================
    def turn_left_or_right(self, target_angle):
        curr_y = self.odom.y
        error = (target_angle - curr_y + 180) % 360 - 180
        if abs(error) < 3: return False
        self.publish(2, v_yaw=0.5 if error > 0 else -0.5)
        time.sleep(0.05)
        return True

    def leg_odometer(self, target_dist, target_angle):
        print(f'里程计位移: {target_dist}')
        start_xyz = list(self.odom.xyz)
        while True:
            curr_xyz = self.odom.xyz
            dist = math.sqrt((curr_xyz[0]-start_xyz[0])**2 + (curr_xyz[1]-start_xyz[1])**2)
            if dist >= target_dist: break
            y_err = (target_angle - self.odom.y + 180) % 360 - 180
            self.publish(7, v_x=0.15, v_yaw=y_err*0.02)
            time.sleep(0.02)

    def gaotai(self):
        print('>>> [执行] 上高台序列 (修正高度与俯仰版)')

        # 1. 预备位：高度给到 0.06 (6厘米)，这是一个能让后腿有下沉空间的“黄金高度”
        # 同时抬头 (rpy_des[1] = 0.3)，把重心后移，给前腿松绑
        for _ in range(30):
            self.publish(0, h=0.08, p_z=0.10)
            self.msg.rpy_des[1] = 0.30  # 抬头
            self.ctrl_lc.publish("robot_control_cmd", self.msg.encode())
            time.sleep(0.02)

        # 2. 执行 22 号动作。注意：custom_pz 绝对不能是 0 ！！！
        # 给 0.06，后腿才有空间往下压
        self.act(22, custom_h=0.23, custom_pz=0.10)


        # 3. 后续动作：跨越、恢复
        self.act(24)
        self.act(0)
        self.act(29)
        self.act(0)

    def chugaotai(self, at_direction):
        print('出高台')
        self.act(25)
        self.act(0)
        
        # ==================== 新增：激光雷达两侧居中操作 ====================
        print('>>> [对齐] 正在执行雷达两侧居中...')
        while True:
            scan = self.lidar.get_scan()
            if scan is None:
                continue
            
            L, F, R, LF, RF = scan
            # 计算左右偏差
            diff = L - R 
            
            # 如果左右误差小于 0.05米（5厘米），则认为已经居中，跳出循环
            if abs(diff) < 0.05:
                print(f'>>> [对齐] 居中完成，当前偏差: {diff:.3f}m')
                break
            
            # 根据偏差方向设定平移速度 v_y
            # 如果 L > R，说明离左墙远，离右墙近，需要向左平移 (v_y > 0)
            # 反之则向右平移 (v_y < 0)
            vy_cmd = 0.08 if diff > 0 else -0.08
            
            # 使用 Gait 7 (标准步态) 进行原地横移 (v_x=0, v_yaw=0)
            self.publish(7, v_x=0.0, v_y=vy_cmd, v_yaw=0.0)
            time.sleep(0.02)
        
        # 居中完成后停止动作，稍微稳一下
        self.publish(0)
        time.sleep(0.2)

        print('朝前')
        while self.turn_left_or_right(at_direction): continue


        # --- 将原本的 self.act(29) 替换为微调前进 2厘米 ---
        print('>>> [位移] 微调前进 2cm')
        self.leg_odometer(0.02, at_direction) 
        # ----------------------------------------------


        self.act(25)
        self.act(0)
        print('朝前')
        while self.turn_left_or_right(at_direction): continue

        # ==================== 新增：前进10cm并实时居中对齐 ====================
        print('>>> [执行] 前进10cm并实时居中...')
        start_xyz = list(self.odom.xyz) # 记录起始位置
        target_dist = 0.10             # 目标 10cm

        while True:
            scan = self.lidar.get_scan()
            curr_xyz = self.odom.xyz
            curr_yaw = self.odom.y

            if scan is None or curr_xyz is None or curr_yaw is None:
                continue

            # 1. 计算移动距离
            dist_moved = math.sqrt((curr_xyz[0]-start_xyz[0])**2 + (curr_xyz[1]-start_xyz[1])**2)
            if dist_moved >= target_dist:
                break

            # 2. 居中逻辑 (L-R 修正)
            L, F, R, LF, RF = scan
            diff_y = L - R
            # 如果偏差大于3cm则进行平移修正
            vy_cmd = 0.08 if diff_y > 0.03 else (-0.08 if diff_y < -0.03 else 0.0)

            # 3. 角度修正 (防止走斜)
            y_err = (at_direction - curr_yaw + 180) % 360 - 180
            vyaw_cmd = y_err * 0.02 # P控制比例项

            # 4. 执行复合动作 (前进 v_x + 修正 v_y + 修正 v_yaw)
            self.publish(7, v_x=0.15, v_y=vy_cmd, v_yaw=vyaw_cmd)
            time.sleep(0.02)

        self.publish(0) # 停止动作
        # ====================================================================

        print('>>> [出高台完成]')

        self.publish(0)

        # ==================== 新增：楼梯辅助函数 ====================
        def louti(self):
            '''
            爬楼梯
            '''
            print('>>> [执行] 爬楼梯关卡 (Gait 31)')
            self.act(0)
            self.act(31)
            print('>>> [完成] 楼梯爬取')



    # ==================== 主流程：Stage 0-5 完全不变，只改 Stage 6 ====================
    def run_competition(self):
        while True:
            scan = self.lidar.get_scan()
            curr_yaw = self.odom.y
            curr_xyz = self.odom.xyz
            
            if scan is None or curr_yaw is None or curr_xyz is None:
                self.publish(0); time.sleep(0.05); continue
            
            dist_from_turn = math.sqrt((curr_xyz[0]-self.turn_done_xyz[0])**2 + (curr_xyz[1]-self.turn_done_xyz[1])**2)
            L, F, R, LF, RF = scan

            if self.stage == 0:
                self.publish(0)
                if self.status.rec_msg.order_process_bar > 0: self.stage = 1

            elif self.stage == 1:
                if (self.task_id == 0 or self.task_id == 3) and F < 0.23: self.stage = 2
                elif self.task_id == 2:
                    if F < 0.3 and (LF < 1.0 or L < 1.0):
                        self.stage = 3; self.sub_stage = 1; self.sub_stage_start_time = time.time()
                elif self.task_id == 4:
                    self.stage = 6 # 进入高台任务
                
                if self.task_id == 1 and dist_from_turn > 0.4: self.task_id = 2
                if self.task_id != 4:
                    vx_target = 0.20 if F > 0.6 else 0.12
                    diff = L - R
                    if self.is_correcting:
                        self.publish(5 if diff > 0 else 6, v_x=0.15, v_y=(0.06 if diff > 0 else -0.06))
                        if abs(diff) < 0.08: self.is_correcting = False
                    else:
                        self.publish(7, v_x=vx_target)
                        if abs(diff) > 0.18: self.is_correcting = True

            elif self.stage == 2: # 左转逻辑
                target_yaw = (curr_yaw + 90) % 360
                while True:
                    error = (target_yaw - self.odom.y + 180) % 360 - 180
                    if abs(error) < 4: break
                    self.publish(2, v_yaw=0.5); time.sleep(0.05)
                self.init_angle = target_yaw
                self.turn_done_xyz = list(self.odom.xyz)
                self.task_id = 1 if self.task_id == 0 else 4
                self.stage = 1

            elif self.stage == 3: # 圆柱逻辑
                yaw_err = (self.init_angle - curr_yaw + 180) % 360 - 180
                if self.sub_stage == 1: 
                    self.publish(7, v_x=0.1, v_y=-0.45, v_yaw=-0.55) 
                    if time.time() - self.sub_stage_start_time > 2.2: self.sub_stage = 2
                elif self.sub_stage == 2: 
                    v_yaw_cmd = 0.70 + (L - 0.65) * 1.5
                    self.publish(7, v_x=0.22, v_yaw=v_yaw_cmd)
                    if RF > 1.8 or R > 1.8: self.sub_stage = 3; self.sub_stage_start_time = time.time()
                elif self.sub_stage == 3: 
                    if abs(yaw_err) > 8: self.publish(7, v_x=0.12, v_yaw=-0.6)
                    else:
                        if time.time() - self.sub_stage_start_time < 1.5: self.publish(7, v_x=0.20)
                        else: self.turn_done_xyz = list(self.odom.xyz); self.stage = 5; self.sub_stage = 0

            elif self.stage == 5: # 脱离圆柱
                if dist_from_turn < 1.5: self.publish(7, v_x=0.25)
                else: self.task_id = 3; self.stage = 1

# ==================== 修改主流程：增加楼梯触发 ====================
    def run_competition(self):
        while True:
            scan = self.lidar.get_scan()
            curr_yaw = self.odom.y
            curr_xyz = self.odom.xyz
            
            if scan is None or curr_yaw is None or curr_xyz is None:
                self.publish(0); time.sleep(0.05); continue
            
            dist_from_turn = math.sqrt((curr_xyz[0]-self.turn_done_xyz[0])**2 + (curr_xyz[1]-self.turn_done_xyz[1])**2)
            L, F, R, LF, RF = scan

            if self.stage == 0:
                self.publish(0)
                if self.status.rec_msg.order_process_bar > 0: self.stage = 1

            elif self.stage == 1:
                # 任务 ID 0-4 的逻辑保持你原来的不变
                if (self.task_id == 0 or self.task_id == 3) and F < 0.23: self.stage = 2
                elif self.task_id == 2:
                    if F < 0.3 and (LF < 1.0 or L < 1.0):
                        self.stage = 3; self.sub_stage = 1; self.sub_stage_start_time = time.time()
                elif self.task_id == 4:
                    self.stage = 6 
                
                # --- 新增：高台任务完成(task_id 5)后，检测楼梯并触发 Stage 7 ---
                elif self.task_id == 5:
                    # 当距离前方楼梯小于 0.25米时，进入爬楼梯阶段
                    if F < 0.25: 
                        print('>>> [探测] 发现楼梯，准备切换 Stage 7')
                        self.stage = 7
                    else:
                        # 没到楼梯前，保持基础巡航前进
                        self.publish(7, v_x=0.15) 

                if self.task_id == 1 and dist_from_turn > 0.4: self.task_id = 2
                
                # 巡航避障保持不变
                if self.task_id != 4 and self.task_id != 5:
                    vx_target = 0.20 if F > 0.6 else 0.12
                    diff = L - R
                    if self.is_correcting:
                        self.publish(5 if diff > 0 else 6, v_x=0.15, v_y=(0.06 if diff > 0 else -0.06))
                        if abs(diff) < 0.08: self.is_correcting = False
                    else:
                        self.publish(7, v_x=vx_target)
                        if abs(diff) > 0.18: self.is_correcting = True






            # ==================== 严格按你要求改动的高台部分开始 ====================
            elif self.stage == 6:
                at_direction = self.init_angle
                print('高台任务')
                # 高台准备
                self.leg_odometer(0.45, at_direction)
                self.act(29)
                self.act(29)
                while self.turn_left_or_right(at_direction):
                    continue
                self.act(0)
                self.gaotai()


                # 第二层高台
                self.act(29)
                self.act(0)
                self.gaotai()
                while self.turn_left_or_right(at_direction):
                    continue
                self.chugaotai(at_direction=at_direction)
                
                # 任务完成
                self.stage = 1
                self.task_id = 5
            # ==================== 严格按你要求改动的高台部分结束 ====================

            # ==================== 新增：楼梯执行阶段 ====================
            elif self.stage == 7:
                at_direction = self.init_angle
                print('>>> [状态] 正在爬楼梯...')
                
                # 执行你提供的楼梯动作
                self.louti()
                
                # 爬完楼梯后，稍微往前走一点脱离楼梯区域
                self.leg_odometer(0.20, at_direction)
                
                # 楼梯完成，进入下一个任务（如果有的话，可以设为 task_id 6）
                print('>>> [系统] 楼梯关卡通过！')
                self.stage = 1
                self.task_id = 6 

def main():
    rclpy.init()
    master = CyberdogRaceMaster(init_angle=0)
    executor = MultiThreadedExecutor()
    executor.add_node(master.lidar)
    Thread(target=executor.spin, daemon=True).start()
    try: master.run_competition()
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()
