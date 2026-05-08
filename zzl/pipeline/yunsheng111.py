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

    def dumuoqiao(self, at_direction):
        print('>>> [状态] 进入独木桥 - 混合传感器分段模式 (v4.0)')
       

        # --- 【第一阶段：起步前严格对齐】 ---
        # 仅在还没开始走的时候，把头转正。一旦开始走，就不再停下校准。
        print('>>> [对齐] 正在执行起步前最后校准...')
        for _ in range(60): # 持续 1.2 秒的强制对齐
            curr_yaw = self.odom.y
            if curr_yaw is None: continue
            y_err = (at_direction - curr_yaw + 180) % 360 - 180
            # 只转头，不迈步
            self.publish(7, v_x=0.0, v_y=0.0, v_yaw=y_err * 0.05, h=0.08, p_z=0.07)
            time.sleep(0.02)

        start_xyz = None
        # 标记是否已经进入第二阶段
        stage_2_triggered = False
        
        while True:
            tl, tr = self.lidar.get_tof() 
            scan = self.lidar.get_scan()
            curr_xyz = self.odom.xyz
            curr_yaw = self.odom.y
            
            if scan is None or curr_xyz is None or curr_yaw is None:
                self.publish(0, h=0.08, p_z=0.07); time.sleep(0.02); continue
            
            if start_xyz is None:
                start_xyz = list(curr_xyz); continue
            
            # 1. 计算移动距离
            dist_moved = math.sqrt((curr_xyz[0]-start_xyz[0])**2 + (curr_xyz[1]-start_xyz[1])**2)
            L, F, R, LF, RF = scan
            y_err = (at_direction - curr_yaw + 180) % 360 - 180

                    # --- 【修正点1：原地对齐逻辑】 ---
            # 你的日志显示起步偏角有11度，这在桥上必掉。
            # 如果偏差大于 3 度，先原地踏步转正，速度 v_x 设为 0
#            if abs(y_err) > 3.0:
#                print(f">>> [修正角度] 当前偏角 {y_err:.1f}度，原地校准中...")
#                self.publish(7, v_x=0.0, v_y=0.0, v_yaw=y_err * 0.05, h=0.08, p_z=0.07)
#                time.sleep(0.02)
#                continue

            # --- 【航向控制：大幅增加 Yaw 权重】 ---
            # 把转头的灵敏度从 0.02 提高到 0.05，确保身子“瞬时”转正
            vyaw_cmd = y_err * 0.05 

            # --- 【分段逻辑开始】 ---
            if dist_moved < 3.5:
                # --- 第一阶段：TOF 精准居中 (0 - 1.5m) ---
                mode_str = "TOF居中"
                # 纠偏逻辑：右减左 (维持你测试成功的方向)
                diff = tr - tl 
                vy_cmd = diff * 0.4  #提高灵敏度系数
                h_val = 0.08  # 标准步高
                p_z_val = 0.07 # 标准底盘高度
            else:
                # --- 第二阶段：雷达居中 + 高抬腿 (1.5m 之后) ---
                if not stage_2_triggered:
                    print(">>> [切换] 到达2m，开启雷达高抬腿模式...")
                    stage_2_triggered = True
                
                mode_str = "雷达高抬腿"
                # 雷达纠偏逻辑：L - R (雷达通常左大右小是偏右，需正vy左移)
                diff = L - R
                vy_cmd = diff * 0.15  # 雷达量程大，增益调小防止晃动
                h_val = 0.09          # 开启高抬腿，跨越小台阶
                p_z_val = 0.07        # 稍微抬高底盘防止踢到台阶

                # 【判定退出】探测到前方墙面
                if F < 0.28:
                    print(f'>>> [探测] 前方距离 {F:.2f}m，发现墙面，退出关卡序列')
                    break
            

            # --- 【核心修正：融合角度修正】 ---
            # 不再执行 continue 拦截，而是将 vyaw_cmd 融合进 publish
            # 减小系数 (0.02)，让转向更柔和，不影响前进
            # --- 角度锁定与安全限制 ---
            #vyaw_cmd = y_err * 0.02

            # --- 【核心优化：身子不直，不准横移】 ---
            # 如果身子偏角大于 2 度，强行减小平移速度 vy，让它优先把身子转正
            if abs(y_err) > 2.0:
                vy_cmd *= 0.3  # 大幅削弱侧移，消除“斜着走”的视觉感

            #安全限制
            vy_cmd = max(min(vy_cmd, 0.04), -0.04)  #限制最大平移，防止侧翻
            #vyaw_cmd = y_err * 0.03             #航向保持权重
            vx_cmd = 0.12 # 保持稳定的前进速度

            # 2. 执行动作
            self.publish(7, v_x=vx_cmd, v_y=vy_cmd, v_yaw=vyaw_cmd, h=h_val, p_z=p_z_val)
            
            # 3. 打印实时日志，每 20 帧打印一次，减少日志压力
            if self.life_count % 20 == 0:
                print(f">>> [{mode_str}] 距离:{dist_moved:.2f}m | F:{F:.2f} | vy:{vy_cmd:.3f} | yaw_err:{y_err:.1f}")
            
            time.sleep(0.02)
        
        # 关卡结束，强制进入左转逻辑
        self.publish(0)
        print('>>> [转向] 准备左转进入下一区域')

    # ==================== 修改 Stage 6 后的自动衔接 ====================
    # 确保你的 run_competition 里的逻辑如下：
    # elif self.stage == 6:
    #     ... 高台 ...
    #     ... 楼梯 ...
    #     self.dumuoqiao(at_direction)
    #     # 独木桥函数一旦 break 出口，立即重置任务，让主循环带入下一次 Stage 2
    #     self.stage = 2 
    #     self.task_id = 3 # 假设3是弯道2的任务 ID


    # ==================== 主运行逻辑 ====================
    # === 将这个新函数粘贴在 run_competition 之前 ===
    def wall_hug_advance(self, side, target_dist, wall_gap=0.35):
        """
        幕布避障工具函数
        side: 'left' 或 'right'
        target_dist: 走多少米切换
        wall_gap: 距离墙面的目标距离(米)
        """
        print(f'>>> [幕布] 贴{side}侧前进 {target_dist}m...')
        start_xyz = list(self.odom.xyz)
        
        while True:
            scan = self.lidar.get_scan()
            curr_xyz = self.odom.xyz
            curr_yaw = self.odom.y
            if scan is None or curr_xyz is None or curr_yaw is None:
                self.publish(0); time.sleep(0.02); continue
            
            # 计算这一段走过的距离
            dist_moved = math.sqrt((curr_xyz[0]-start_xyz[0])**2 + (curr_xyz[1]-start_xyz[1])**2)
            if dist_moved >= target_dist:
                break
            
            # 航向锁定 (解决身子斜的关键)
            y_err = (self.init_angle - curr_yaw + 180) % 360 - 180
            vyaw_cmd = y_err * 0.05 
            
            # 贴墙逻辑
            current_gap = scan[0] if side == 'left' else scan[2] # scan[0]=L, scan[2]=R
            gap_error = current_gap - wall_gap
            
            if side == 'right':
                vy_cmd = -gap_error * 0.4 # 右侧大说明偏左，vy取负向右移
            else:
                vy_cmd = gap_error * 0.4  # 左侧大说明偏右，vy取正向左移

            # 核心优化：身子不直不准平移
            if abs(y_err) > 1.5:
                vy_cmd *= 0.2
                vx_cmd = 0.08 
            else:
                vx_cmd = 0.15

            # 安全限速与发布
            vy_cmd = max(min(vy_cmd, 0.05), -0.05)
            self.publish(7, v_x=vx_cmd, v_y=vy_cmd, v_yaw=vyaw_cmd)
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

                        # ==================== Stage 6: 高台直连楼梯流程 ====================
            elif self.stage == 6:
                at_direction = self.init_angle
                print('高台任务执行')
                # 1. 高台准备
                self.leg_odometer(0.20, at_direction)
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
               
            elif self.stage == 6:
                # ... 前面的高台、楼梯、独木桥逻辑保持不变 ...
                
                # --- 这里是你刚刚调通的左转逻辑 ---
                print('>>> [转向] 独木桥结束，执行强制 90 度左转')
                target_yaw = (self.odom.y + 90) % 360
                while True:
                    curr_y = self.odom.y
                    if curr_y is None: continue
                    error = (target_yaw - curr_y + 180) % 360 - 180
                    if abs(error) < 4: break
                    self.publish(2, v_yaw=0.5) 
                    time.sleep(0.05)
                
                # 更新角度，这步非常关键！
                self.init_angle = target_yaw 
                self.publish(0); time.sleep(0.5) 

                # ==================== 从这里开始添加幕布逻辑 ====================
                # 根据你的描述：左转后依次是 右贴墙 -> 左贴墙 -> 右贴墙
                # 设定贴墙距离为 0.35m，每段 1.5m
                GAP_VAL = 0.35 

                # 1. 第一个幕布 (贴右)
                self.wall_hug_advance(side='right', target_dist=1.5, wall_gap=GAP_VAL)
                
                # 2. 第二个幕布 (贴左)
                self.wall_hug_advance(side='left', target_dist=1.5, wall_gap=GAP_VAL)
                
                # 3. 第三个幕布 (贴右)
                self.wall_hug_advance(side='right', target_dist=1.5, wall_gap=GAP_VAL)

                print('>>> [完成] 🎉 幕布关卡全部通过！')

                # ===============================================

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

