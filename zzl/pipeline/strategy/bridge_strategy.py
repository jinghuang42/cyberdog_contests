"""
独木桥策略
"""
import time
import math
from .base_strategy import StageStrategy, StrategyContext


class BridgeStrategy(StageStrategy):
    """独木桥策略"""

    def __init__(self, direction: float):
        super().__init__()
        self._direction = direction
        self._stage_2_triggered = False

    def name(self) -> str:
        return "独木桥"

    def enter(self, ctx: StrategyContext) -> None:
        super().enter(ctx)
        self._stage_2_triggered = False

    def is_complete(self, ctx: StrategyContext) -> bool:
        """独木桥完成判断"""
        scan = ctx.sensor_mgr.lidar.get_scan()
        if scan is None:
            return False
        return self._phase >= 2 and scan.front < 0.40

    def execute(self, ctx: StrategyContext) -> None:
        executor = ctx.executor
        sensor = ctx.sensor_mgr
        config = ctx.config

        state = sensor.get_state()
        if state.position is None or state.yaw is None or state.scan is None:
            executor.exec_gait(0, step_height=0.08, pitch_z=0.07)
            return

        # 计算移动距离
        start_xyz = ctx.task_data.get('start_xyz', state.position)
        if 'start_xyz' not in ctx.task_data:
            ctx.task_data['start_xyz'] = state.position

        dist_moved = math.sqrt(
            (state.position[0] - start_xyz[0]) ** 2 +
            (state.position[1] - start_xyz[1]) ** 2
        )

        scan = state.scan
        y_err = config.yaw_error(state.yaw, self._direction)

        if self._phase == 0:
            # 阶段0: 原地对齐航向
            for _ in range(60):
                curr_yaw = sensor.odom.yaw
                if curr_yaw is None:
                    continue
                y_err = config.yaw_error(curr_yaw, self._direction)
                executor.exec_gait(7, v_x=0.0, v_y=0.0, v_yaw=y_err * 0.08,
                                  step_height=0.08, pitch_z=0.07)
                time.sleep(0.02)
            self._phase = 1

        elif self._phase == 1:
            # 阶段1: 直行
            if dist_moved >= config.BRIDGE_EXIT_M:
                if not self._stage_2_triggered:
                    # 触发雷达矫正
                    self._radar_correction(ctx)
                    self._stage_2_triggered = True
                executor.exec_gait(7, v_x=config.STRAIGHT_SPEED_M_S,
                                  step_height=config.HIGH_STEP_HEIGHT, pitch_z=0.07)

                if scan.front < 0.40:
                    self._phase = 2
            else:
                # 正常直行
                if abs(y_err) > 1.0:
                    vx, vy = 0.05, 0.0
                else:
                    vx = 0.10
                    vy = max(min(scan.center_diff * 0.10, 0.03), -0.03)

                executor.exec_gait(7, v_x=vx, v_y=vy, v_yaw=y_err * 0.10,
                                  step_height=config.DEFAULT_STEP_HEIGHT, pitch_z=0.07)

            if sensor.life_count % 15 == 0:
                print(f"[独木桥] 距起:{dist_moved:.2f}m | L-R:{scan.center_diff:.2f} | "
                      f"偏角:{y_err:.1f} | vy:{vy:.3f}")

        elif self._phase == 2:
            # 阶段2: 完成
            executor.exec_gait(0)
            print(">>> [独木桥] 通行完毕")

    def _radar_correction(self, ctx: StrategyContext) -> None:
        """雷达姿态矫正"""
        executor = ctx.executor
        sensor = ctx.sensor_mgr
        config = ctx.config

        print(">>> [独木桥] 利用雷达数据强制回正身体...")
        for _ in range(50):
            scan = sensor.lidar.get_scan()
            curr_yaw = sensor.odom.yaw
            if scan is None or curr_yaw is None:
                continue

            # 雷达矫正算法
            l_err_raw = (scan.left_front - scan.left) - (scan.right_front - scan.right)
            l_err = l_err_raw if math.isfinite(l_err_raw) else 0.0

            odom_err = config.yaw_error(curr_yaw, self._direction)
            total_yaw_cmd = odom_err * 0.08 + l_err * 0.5

            executor.exec_gait(7, v_x=0.0, v_y=0.0, v_yaw=total_yaw_cmd,
                              step_height=0.08, pitch_z=0.07)
            time.sleep(0.02)

        print(">>> [独木桥] 身体已回正，开启高抬腿...")
