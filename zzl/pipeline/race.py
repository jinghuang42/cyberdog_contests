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

        # --- 第一步：强制提拉（解决后腿勾住的关键） ---
        # 31 结束后，先不停，用高抬腿动作往前冲 2 秒
        # 这样能确保即使 31 没做完，后腿也能被硬拽上来
        print('>>> [提拉] 正在执行后肢提拉，防止勾住台阶边缘...')
        for _ in range(100): # 100次 * 0.02s = 2秒
            # v_x=0.18 (给向前的力), h=0.13 (步高必须足够高才能跨过边缘)
            self.publish(32, v_x=0.18, h=0.13, p_z=0.08)
            time.sleep(0.02)

        # --- 第二步：你的核心要求：原地站立等待 3 秒 ---
        print('>>> [等待] 提拉完成，原地站立稳定 3 秒...')
        for i in range(150): # 150次循环 * 0.02秒 = 3秒
            # 持续发布 Gait 0 (站立指令)，确保重心稳定
            self.publish(0, h=0.08, p_z=0.08)
            if i % 50 == 0:
                print(f">>> [稳定中] 已等待 {i*0.02:.1f}s...")
            time.sleep(0.02)

        print('>>> [稳定] 3秒等待结束，开始进入独木桥')




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
        print('>>> [执行] 前进 5.4cm 并实时居中...')
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
            if dist_moved >= 0.54: # 5cm
                break

            L, F, R, LF, RF = scan
            diff_y = L - R
            vy_cmd = 0.08 if diff_y > 0.03 else (-0.08 if diff_y < -0.03 else 0.0)
            y_err = (at_direction - curr_yaw + 180) % 360 - 180
            # 执行前进
            self.publish(7, v_x=0.15, v_y=vy_cmd, v_yaw=y_err*0.02)
            time.sleep(0.02)
        self.publish(0)

    def dumuoqiao(self, at_direction=None):
        """
        独木桥 v7.0 修正版 (仅在第二阶段增加雷达姿态强制矫正)
        """
        import math
        import time # 确保导入 time
        
        # --- 【第一阶段：目标航向自动锁定】 ---
        if at_direction is None:
            at_direction = self.odom.y
        
        print(f'>>> [状态] 进入独木桥 - 目标航向设定为: {at_direction:.2f}')

        # 强制原地对齐目标角度
        for i in range(60):
            curr_yaw = self.odom.y
            if curr_yaw is None: continue
            y_err = (at_direction - curr_yaw + 180) % 360 - 180
            self.publish(7, v_x=0.0, v_y=0.0, v_yaw=y_err * 0.08, h=0.08, p_z=0.07)
            time.sleep(0.02)

        start_xyz = None
        stage_2_triggered = False
        
        while True:
            scan = self.lidar.get_scan()
            curr_xyz, curr_yaw = self.odom.xyz, self.odom.y
            
            if scan is None or curr_xyz is None or curr_yaw is None:
                self.publish(0, h=0.08, p_z=0.07); time.sleep(0.02); continue
            
            if start_xyz is None:
                start_xyz = list(curr_xyz); continue
            
            # 距离与雷达处理
            dist_moved = math.sqrt((curr_xyz[0]-start_xyz[0])**2 + (curr_xyz[1]-start_xyz[1])**2)
            L_raw, F, R_raw, LF_raw, RF_raw = scan
            
            # 数据清洗，防止 inf 导致趴下
            L = L_raw if math.isfinite(L_raw) else 1.2
            R = R_raw if math.isfinite(R_raw) else 1.2
            LF = LF_raw if math.isfinite(LF_raw) else 1.5
            RF = RF_raw if math.isfinite(RF_raw) else 1.5
            diff = L - R

            # 航向误差计算
            y_err = (at_direction - curr_yaw + 180) % 360 - 180 

            # --- 【核心逻辑】 ---
            vyaw_cmd = y_err * 0.10 
            
            if abs(y_err) > 1.0:
                vx_cmd, vy_cmd = 0.05, 0.0
                status_note = "强制矫正身体"
            else:
                vx_cmd = 0.10
                vy_cmd = max(min(diff * 0.10, 0.03), -0.03) 
                status_note = "航向准确-居中"

            # --- 【第二阶段：此处增加雷达矫正】 ---
            if dist_moved >= 3.5:
                if not stage_2_triggered:
                    print(">>> [切换] 到达3.5m，正在利用雷达数据强制回正身体...")
                    # 暂停 1 秒，结合 Odom 和 Lidar 进行二次对齐
                    for _ in range(50):
                        s = self.lidar.get_scan()
                        cy = self.odom.y
                        if s is None or cy is None: continue
                        
                        # 【雷达矫正算法】：比较左右前侧和正侧方的距离差
                        # 如果 (LF - L) > (RF - R)，说明狗头偏右了，需要向左转
                        l_err_raw = (s[3] - s[0]) - (s[4] - s[2])
                        l_err = l_err_raw if math.isfinite(l_err_raw) else 0.0
                        
                        # 融合 Odom 角度误差
                        odom_err = (at_direction - cy + 180) % 360 - 180
                        
                        # 综合计算转向力：10.0 为雷达补偿系数
                        total_yaw_cmd = odom_err * 0.08 + (l_err * 0.5)
                        
                        self.publish(7, 0, 0, total_yaw_cmd, 0.08, 0.07)
                        time.sleep(0.02)
                    
                    print(">>> [切换] 身体已完全回正，开启高抬腿...")
                    stage_2_triggered = True
                
                h_val, p_z_val = 0.10, 0.07 # 开启高抬腿
                if F < 0.40: break
            else:
                h_val, p_z_val = 0.08, 0.07

            # 发布动作指令
            self.publish(7, v_x=vx_cmd, v_y=vy_cmd, v_yaw=vyaw_cmd, h=h_val, p_z=p_z_val)
            
            if self.life_count % 15 == 0:
                print(f"[{status_note}] 距起:{dist_moved:.2f}m | L-R:{diff:.2f} | 偏角:{y_err:.1f} | vy:{vy_cmd:.3f}")
            
            time.sleep(0.02)
        
        self.publish(0)
        print('>>> [成功] 独木桥通行完毕')
    # ==================== 主运行逻辑 ====================
    def wall_hug_advance(self, side, target_dist, wall_gap=0.30, stop_at_wall=False, switch_dist=None):
        import math
        import time

        # --- 1. 强制起步定身 ---
        for _ in range(35):
            self.publish(0, h=0.08, p_z=0.07); time.sleep(0.02)

        # --- 2. 原地侧移对齐 ---
        print(f'>>> [幕布-位移] 正在向{side}平移对齐...')
        align_start = time.time() # 记录对齐开始时间
        
        while True:
            scan = self.lidar.get_scan()
            if scan is None: 
                self.publish(0, h=0.08, p_z=0.07) # 维持心跳，防止趴下
                time.sleep(0.02)
                continue
            
            # --- 【修正1：安全获取距离数据】 ---
            raw_gap = scan[0] if side == 'left' else scan[2]
            # 如果扫到无穷远，假设距离为 1.0m，防止计算报错
            current_gap = raw_gap if math.isfinite(raw_gap) else 1.0
            
            # --- 【修正2：加入超时强制退出，防止原地卡死趴下】 ---
            y_err = (self.init_angle - self.odom.y + 180) % 360 - 180
            
            # 如果距离对准了，或者原地耗时超过 3 秒，强制起步
            if abs(current_gap - wall_gap) < 0.05 or (time.time() - align_start > 3.0): 
                print(f">>> [对齐完成] 最终距离: {current_gap:.2f}")
                break
            
            vy_cmd = (current_gap - wall_gap) * 0.3 if side == 'left' else -(current_gap - wall_gap) * 0.3
            self.publish(7, v_x=0.0, v_y=max(min(vy_cmd, 0.05), -0.05), v_yaw=y_err * 0.06, h=0.08, p_z=0.07)
            time.sleep(0.02)

        # --- 3. 贴墙前进 ---
        print(f'>>> [幕布-前进] 起步...')
        start_xyz = list(self.odom.xyz)
        while True:
            scan = self.lidar.get_scan()
            curr_xyz, curr_yaw = self.odom.xyz, self.odom.y
            if scan is None or curr_xyz is None or curr_yaw is None:
                self.publish(0, h=0.08, p_z=0.07); time.sleep(0.02); continue
            
            dist_moved = math.sqrt((curr_xyz[0]-start_xyz[0])**2 + (curr_xyz[1]-start_xyz[1])**2)
            L_raw, F, R_raw, LF, RF = scan
            
            # --- 【修正3：数据清洗】 ---
            L = L_raw if math.isfinite(L_raw) else 1.2
            R = R_raw if math.isfinite(R_raw) else 1.2

            # --- 判定退出 ---
            if stop_at_wall:
                if dist_moved >= target_dist and F < 0.30:
                    print(f'>>> [终点探测] 距离墙面 {F:.2f}m，准备转向')
                    break
            else:
                if dist_moved >= target_dist: break
            
            # --- 模式切换逻辑 ---
            if switch_dist and dist_moved >= switch_dist:
                diff_y = L - R
                vy_cmd = diff_y * 0.15 
                mode_label = "雷达居中"
            else:
                current_gap = L if side == 'left' else R
                vy_cmd = (current_gap - wall_gap) * 0.35 if side == 'left' else -(current_gap - wall_gap) * 0.35
                mode_label = side + "贴墙"

            # 航向锁定
            y_err = (self.init_angle - curr_yaw + 180) % 360 - 180
            vyaw_cmd = y_err * 0.06
            
            # 这里的逻辑很好，保留：身子歪了禁止横移
            if abs(y_err) > 1.5:
                vy_cmd, vx_cmd = 0.0, 0.07 
            else:
                vx_cmd = 0.15

            # --- 【修正4：最终输出限幅】 ---
            safe_vy = max(min(vy_cmd, 0.04), -0.04)
            self.publish(7, v_x=vx_cmd, v_y=safe_vy, v_yaw=vyaw_cmd, h=0.08, p_z=0.07)
            
            if self.life_count % 20 == 0:
                print(f">>> [{mode_label}] 距离:{dist_moved:.2f}m | F:{F:.2f} | 偏角:{y_err:.1f}")
            time.sleep(0.02)
            
        self.publish(0)

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

            # ==================== Stage 6: 全自动关卡链 (唯一入口) ====================
            elif self.stage == 6:
                at_direction = self.init_angle
                
                # 1. 高台准备
                print('>>> [1. 高台阶段] 启动')
                self.leg_odometer(0.20, at_direction)
                self.act(29); self.act(29)
                while self.turn_left_or_right(at_direction): continue
                self.act(0)
                
                # 2. 上两层高台
                self.gaotai() # 第一层
                self.act(29); self.act(0)
                self.gaotai() # 第二层
                
                # 3. 出高台 (含5cm前进)
                while self.turn_left_or_right(at_direction): continue
                self.chugaotai(at_direction=at_direction)
                
                # 4. 爬楼梯逻辑
                print('>>> [2. 楼梯阶段] 启动')
                self.louti(at_direction)

                # 5. 进入独木桥
                print('>>> [3. 独木桥阶段] 启动')
                self.dumuoqiao(at_direction)
               
                # --- 【核心修复：紧接在独木桥后执行转向】 ---
                print('>>> [4. 转向阶段] 执行强制 90 度左转')
                target_yaw = (self.odom.y + 90) % 360
                while True:
                    curr_y = self.odom.y
                    if curr_y is None: continue
                    error = (target_yaw - curr_y + 180) % 360 - 180
                    if abs(error) < 4: break # 精度 4 度
                    self.publish(2, v_yaw=0.5) 
                    time.sleep(0.05)
                
                                # 【极其重要】更新全局基准角度，否则后面的幕布逻辑会撞墙
                self.init_angle = target_yaw 
                self.turn_done_xyz = list(self.odom.xyz)
                self.publish(0, h=0.08, p_z=0.07); time.sleep(0.5) 

                # --- [6. 幕布阶段] (右 -> 左 -> 右) ---
                # ==================== 幕布关卡串联 (Stage 6 结尾) ====================
                # ==================== 幕布关卡 ====================
                GAP_VAL = 0.30 

                # 1. 幕布 1：贴右墙走 1.2m
                self.wall_hug_advance(side='right', target_dist=1.2, wall_gap=GAP_VAL)
                
                # 2. 幕布 2：贴左墙走 1.0m
                self.wall_hug_advance(side='left',  target_dist=1.0, wall_gap=GAP_VAL)
                
                # 3. 幕布 3：贴右走 1.1m，随后切换雷达居中，直到离墙 0.3m 停止
                # 我们传入 switch_dist=1.1
                self.wall_hug_advance(side='right', target_dist=1.1, wall_gap=GAP_VAL, stop_at_wall=True, switch_dist=1.1)

                # ==================== 最终冲刺序列 ====================
                print('>>> [终点转向] 执行 90 度左转...')
                target_yaw = (self.odom.y + 90) % 360
                while True:
                    curr_y = self.odom.y
                    if curr_y is None: continue
                    if abs((target_yaw - curr_y + 180) % 360 - 180) < 3: break
                    self.publish(2, v_yaw=0.55)
                    time.sleep(0.05)
                
                self.init_angle = target_yaw # 更新航向
                self.publish(0, h=0.08, p_z=0.07); time.sleep(0.5)

                # 4. 前进并居中走 1.0m
                print('>>> [终点冲刺] 居中前进 1.0m...')
                start_dash = list(self.odom.xyz)
                while True:
                    scan = self.lidar.get_scan()
                    curr_xyz, curr_yaw = self.odom.xyz, self.odom.y
                    if scan is None or curr_xyz is None: continue
                    
                    dash_dist = math.sqrt((curr_xyz[0]-start_dash[0])**2 + (curr_xyz[1]-start_dash[1])**2)
                    if dash_dist >= 1.0: break # 走完1米
                    
                    # 居中逻辑 (L - R)
                    diff_dash = scan[0] - scan[2]
                    y_dash_err = (self.init_angle - curr_yaw + 180) % 360 - 180
                    
                    # 稳定前进
                    self.publish(7, v_x=0.15, v_y=max(min(diff_dash*0.2, 0.04), -0.04), v_yaw=y_dash_err*0.05)
                    time.sleep(0.02)

                # 5. 任务圆满结束：站立 -> 趴下
                print('>>> [任务圆满完成] 正在保存姿态并停止...')
                # 站稳
                self.act(0)
                time.sleep(1.0)
                
                # 趴下 (Mode 1 是小米的掉电/放松模式)
                print('>>> [系统] 趴下。再见！')
                for _ in range(10):
                    self.msg.mode = 1
                    self.msg.gait_id = 0
                    self.msg.life_count = self.life_count % 128
                    self.life_count += 1
                    self.ctrl_lc.publish("robot_control_cmd", self.msg.encode())
                    time.sleep(0.02)
                
                self.stage = 1
                self.task_id = 100 # 结束
                


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

