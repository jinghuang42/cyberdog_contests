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

    # ==================== 基础函数（保持不变） ====================
    def publish(self, gait_idx, v_x=None, v_y=None, v_yaw=None, h=0.08, p_z=0.035):
        try:
            step = self.steps["step"][int(gait_idx)]
            self.msg.mode, self.msg.gait_id = step["mode"], step["gait_id"]
            self.msg.contact, self.msg.value, self.msg.duration = step["contact"], step["value"], step["duration"]
            vx = v_x if v_x is not None else step["vel_des"][0]
            vy = v_y if v_y is not None else step["vel_des"][1]
            vyaw = v_yaw if v_yaw is not None else step["vel_des"][2]
            self.msg.vel_des = [vx, vy, vyaw]
            self.msg.step_height, self.msg.pos_des = [h, h], [0.0, 0.0, p_z]
            
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
        print(f">>> [动作] 执行 Gait {gait_idx}")
        time.sleep(0.1)
        while self.status.rec_msg.order_process_bar >= 100:
            self.publish(gait_idx, h=custom_h, p_z=custom_pz); time.sleep(0.02)
        while self.status.rec_msg.order_process_bar < 98:
            self.publish(gait_idx, h=custom_h, p_z=custom_pz); time.sleep(0.02)

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

    # ==================== 成功的关卡动作函数 ====================
    def gaotai(self):
        print('>>> [执行] 上高台序列 (修正高度与俯仰版)')
        for _ in range(30):
            self.publish(0, h=0.08, p_z=0.10)
            self.msg.rpy_des[1] = 0.30  # 抬头
            self.ctrl_lc.publish("robot_control_cmd", self.msg.encode())
            time.sleep(0.02)
        self.act(22, custom_h=0.23, custom_pz=0.10)
        self.act(24)
        self.act(0)
        self.act(29)
        self.act(0)

    def louti(self, at_direction):
        '''
        爬楼梯关卡逻辑
        '''
        print('>>> [执行] 爬楼梯步态 31')
        self.act(0)   # 爬楼梯前站稳
        self.act(31)  # 执行爬楼梯步态
        
        # --- 核心要求：执行完31后，原地站立等待3秒 ---
        print('>>> [等待] 步态31执行完毕，原地站立稳定 3 秒...')
        for i in range(150): # 150次循环 * 0.02秒 = 3秒
            # 持续发布 Gait 0 (站立指令)，确保机器狗不趴下，保持重心稳定
            self.publish(0, h=0.08, p_z=0.08)
            if i % 50 == 0:
                print(f">>> [稳定中] 已等待 {i*0.02:.1f}s...")
            time.sleep(0.02)
            
        print('>>> [稳定] 3秒等待结束，开始进入独木桥')

    def dumuoqiao(self, at_direction):
        print('>>> [状态] 进入独木桥 - 准备站稳')
        
        # --- 【核心新增：强制站稳期】 ---
        # 在做任何移动前，先强制原地站立 2 秒 (100次循环 * 0.02s)
        # 这一步能让爬完楼梯后的身体晃动彻底停止，并重置 WBC 控制器
        print('>>> [稳态] 正在强制稳固重心，请稍候...')
        for _ in range(100): 
            self.publish(0, h=0.08, p_z=0.08) # 维持高位站姿，稳住不动
            time.sleep(0.02)
        
        # 站稳后再初始化变量
        start_xyz = None
        target_dist = 5.0
        
        print('>>> [起步] 重心已稳，开始通行独木桥')
        while True:
            scan = self.lidar.get_scan() 
            curr_xyz = self.odom.xyz
            curr_yaw = self.odom.y
            
            # 心跳维持
            if scan is None or curr_xyz is None or curr_yaw is None:
                self.publish(0, h=0.08, p_z=0.08)
                time.sleep(0.02)
                continue
            
            # 站稳后才记录起始坐标，这样位移计算最准
            if start_xyz is None:
                start_xyz = list(curr_xyz)
                print(f'>>> [坐标锁定] 起始点: {start_xyz}')
                continue
            
            # 距离计算
            dist_moved = math.sqrt((curr_xyz[0]-start_xyz[0])**2 + (curr_xyz[1]-start_xyz[1])**2)
            
            # 居中逻辑
            L, F, R, LF, RF = scan
            diff_y = L - R
            vy_cmd = 0.05 if diff_y > 0.02 else (-0.05 if diff_y < -0.02 else 0.0)
            
            # 航向修正
            y_err = (at_direction - curr_yaw + 180) % 360 - 180
            vyaw_cmd = y_err * 0.02
            
            # 到达终点
            if dist_moved >= target_dist:
                print('>>> [完成] 独木桥通行完毕')
                break
                
            # --- 【平稳起步】 ---
            # 如果刚开始走（前0.5米），速度稍微慢一点(0.15)，之后再加速到(0.22)
            current_vx = 0.15 if dist_moved < 0.5 else 0.22
            
            self.publish(7, v_x=current_vx, v_y=vy_cmd, v_yaw=vyaw_cmd, h=0.10, p_z=0.08)
            
            if int(dist_moved * 100) % 20 == 0:
                print(f'>>> [进度] 位移: {dist_moved:.2f}m / 5.0m')
            
            time.sleep(0.02)
        
        self.publish(0)

    def chugaotai(self, at_direction):
        print('出高台')
        self.act(25)
        self.act(0)
        
        # 居中对齐
        print('>>> [对齐] 正在执行雷达两侧居中...')
        while True:
            scan = self.lidar.get_scan()
            if scan is None: continue
            L, F, R, LF, RF = scan
            diff = L - R 
            if abs(diff) < 0.05: break
            vy_cmd = 0.08 if diff > 0 else -0.08
            self.publish(7, v_x=0.0, v_y=vy_cmd, v_yaw=0.0)
            time.sleep(0.02)
        
        self.publish(0); time.sleep(0.2)
        print('朝前')
        while self.turn_left_or_right(at_direction): continue

        print('>>> [位移] 微调前进 2cm')
        self.leg_odometer(0.02, at_direction) 

        self.act(25)
        self.act(0)
        print('朝前')
        while self.turn_left_or_right(at_direction): continue


# --- 前进 5cm 并实时居中（修复不动的问题） ---
        print('>>> [执行] 前进 5cm 并实时居中...')
        start_xyz = None # 改用延迟初始化，确保位置准确
        while True:
            scan = self.lidar.get_scan()
            curr_xyz = self.odom.xyz
            curr_yaw = self.odom.y

            # 核心修复：如果数据没读到，也要发 publish(0) 维持心跳，否则狗会发呆不动
            if scan is None or curr_xyz is None or curr_yaw is None:
                self.publish(0) 
                time.sleep(0.02)
                continue

            if start_xyz is None:
                start_xyz = list(curr_xyz) # 在这里记录起始点
                continue

            dist_moved = math.sqrt((curr_xyz[0]-start_xyz[0])**2 + (curr_xyz[1]-start_xyz[1])**2)
            if dist_moved >= 0.50: # 5cm
                break

            L, F, R, LF, RF = scan
            diff_y = L - R
            vy_cmd = 0.08 if diff_y > 0.03 else (-0.08 if diff_y < -0.03 else 0.0)
            y_err = (at_direction - curr_yaw + 180) % 360 - 180
            # 执行前进
            self.publish(7, v_x=0.15, v_y=vy_cmd, v_yaw=y_err*0.02)
            time.sleep(0.02)
        self.publish(0)


    def dumuoqiao(self, at_direction):
        print('>>> [状态] 进入独木桥 - 暴力推进模式')
        
        # 1. 初始化变量
        start_xyz = None
        target_dist = 3.0
        
        while True:
            scan = self.lidar.get_scan() 
            curr_xyz = self.odom.xyz
            curr_yaw = self.odom.y
            
            # 保命心跳逻辑
            if scan is None or curr_xyz is None or curr_yaw is None:
                self.publish(0, h=0.08, p_z=0.06) 
                time.sleep(0.02)
                continue
            
            # 延迟初始化起始坐标
            if start_xyz is None:
                start_xyz = list(curr_xyz)
                print(f'>>> [独木桥] 坐标锁定，起始点: {start_xyz}')
                continue
            
            # 计算移动距离
            dist_moved = math.sqrt((curr_xyz[0]-start_xyz[0])**2 + (curr_xyz[1]-start_xyz[1])**2)
            
            # 打印距离
            print(f'>>> [进度] 物理位移: {dist_moved:.2f}m / 5.0m')

            # 居中逻辑 (L-R 差值)
            L, F, R, LF, RF = scan
            diff_y = L - R
            # 独木桥微调平移速度，减小 vy，防止横向摆动太大导致滑落
            vy_cmd = 0.04 if diff_y > 0.02 else (-0.04 if diff_y < -0.02 else 0.0)
            
            # 航向修正
            y_err = (at_direction - curr_yaw + 180) % 360 - 180
            vyaw_cmd = y_err * 0.02
            
            # 距离达标判断
            if dist_moved >= target_dist:
                print('>>> [完成] 独木桥通行完毕')
                break
                
            # --- 【核心改进】 ---
            # 1. 使用 Gait 32 (你通关高台和楼梯提拉用的步态，力量大)
            # 2. 增加前进速度 v_x = 0.22 (确保能冲起来)
            # 3. 增加步高 h = 0.11 (确保脚不蹭桥面)
            # 4. 增加底盘高度 p_z = 0.06 (确保肚子不蹭桥面)
            self.publish(32, v_x=0.22, v_y=vy_cmd, v_yaw=vyaw_cmd, h=0.11, p_z=0.06)
            time.sleep(0.02)
        
        self.publish(0)


    # ==================== 主运行逻辑 ====================
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
                self.publish(0); 
                if self.status.rec_msg.order_process_bar > 0: self.stage = 1

            elif self.stage == 1:
                if (self.task_id == 0 or self.task_id == 3) and F < 0.23: self.stage = 2
                elif self.task_id == 2:
                    if F < 0.3 and (LF < 1.0 or L < 1.0):
                        self.stage = 3; self.sub_stage = 1; self.sub_stage_start_time = time.time()
                elif self.task_id == 4:
                    self.stage = 6 

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

            elif self.stage == 2: # 转弯逻辑
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

            # ==================== Stage 6: 高台直连楼梯流程 ====================
            elif self.stage == 6:
                at_direction = self.init_angle
                print('高台任务执行')
                # 1. 高台准备
                self.leg_odometer(0.45, at_direction)
                self.act(29)
                self.act(29)
                while self.turn_left_or_right(at_direction): continue
                self.act(0)
                
                # 2. 上两层高台
                self.gaotai() # 第一层
                self.act(29)
                self.act(0)
                self.gaotai() # 第二层
                
                # 3. 出高台 (内含5cm对齐前进)
                while self.turn_left_or_right(at_direction): continue
                self.chugaotai(at_direction=at_direction)
                
                # 4. 直连爬楼梯逻辑
                self.louti(at_direction)
                # 4. 立即进入独木桥
                self.dumuoqiao(at_direction)
                
                # 5. 任务完成，返回巡航或结束
                print('>>> [任务] 高台与楼梯全部通过')
                self.stage = 1
                self.task_id = 9

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
