"""
高台策略
"""
import time
from .base_strategy import StageStrategy, StrategyContext


class GaotaiStrategy(StageStrategy):
    """高台策略 - 上高台和出高台"""

    def __init__(self):
        super().__init__()
        self._lift_start_time = 0.0
        self._stable_start_time = 0.0

    def name(self) -> str:
        return "高台"

    def enter(self, ctx: StrategyContext) -> None:
        super().enter(ctx)
        self._lift_start_time = 0.0
        self._stable_start_time = 0.0

    def is_complete(self, ctx: StrategyContext) -> bool:
        """高台完成判断"""
        return ctx.sensor_mgr.status.progress > ctx.config.GAIT_PROGRESS

    def execute(self, ctx: StrategyContext) -> None:
        """执行高台动作序列"""
        executor = ctx.executor
        config = ctx.config

        if self._phase == 0:
            # 阶段0: 抬头准备
            for _ in range(30):
                cmd = ctx.sensor_mgr.lidar  # 保持心跳
                executor.exec_gait(0, step_height=0.08, pitch_z=0.10)
                ctx.sensor_mgr.lidar  # 引用防止GC
                time.sleep(0.02)

            # 切换到阶段1
            self._phase = 1

        elif self._phase == 1:
            # 阶段1: 上高台
            executor.act(22, step_height=config.GAOTAI_STEP_HEIGHT, pitch_z=0.10,
                        status_monitor=ctx.sensor_mgr.status)
            executor.act(24, status_monitor=ctx.sensor_mgr.status)
            executor.act(0, status_monitor=ctx.sensor_mgr.status)
            executor.act(29, status_monitor=ctx.sensor_mgr.status)
            executor.act(0, status_monitor=ctx.sensor_mgr.status)
            self._phase = 2

        elif self._phase == 2:
            # 阶段2: 等待完成
            pass


class ChugaotaiStrategy(StageStrategy):
    """出高台策略"""

    def __init__(self, target_direction: float):
        super().__init__()
        self._target_direction = target_direction
        self._stage = 0

    def name(self) -> str:
        return "出高台"

    def is_complete(self, ctx: StrategyContext) -> bool:
        if self._stage >= 4:
            return True
        # 检查进度
        return ctx.sensor_mgr.status.progress > 95 and self._stage >= 4

    def execute(self, ctx: StrategyContext) -> None:
        executor = ctx.executor
        config = ctx.config
        sensor = ctx.sensor_mgr

        if self._stage == 0:
            # 出高台动作
            executor.act(25, status_monitor=sensor.status)
            executor.act(0, status_monitor=sensor.status)
            self._stage = 1

        elif self._stage == 1:
            # 雷达居中
            while True:
                scan = sensor.lidar.get_scan()
                if scan is None:
                    executor.exec_gait(0)
                    continue

                diff = scan.center_diff
                if abs(diff) < config.CENTER_TOLERANCE_M:
                    break

                vy_cmd = config.LATERAL_SPEED_M_S if diff > 0 else -config.LATERAL_SPEED_M_S
                executor.exec_gait(7, v_y=vy_cmd)
                time.sleep(0.02)

            executor.exec_gait(0)
            time.sleep(0.2)
            self._stage = 2

        elif self._stage == 2:
            # 转向目标方向
            self._turn_to_direction(ctx, self._target_direction)
            self._stage = 3

        elif self._stage == 3:
            # 微调前进
            self._leg_odometer(ctx, 0.02, self._target_direction)
            executor.act(25, status_monitor=sensor.status)
            executor.act(0, status_monitor=sensor.status)
            self._stage = 4

        elif self._stage == 4:
            # 再转向
            self._turn_to_direction(ctx, self._target_direction)

    def _turn_to_direction(self, ctx: StrategyContext, target: float) -> None:
        """转向到目标方向"""
        executor = ctx.executor
        sensor = ctx.sensor_mgr
        config = ctx.config

        while True:
            curr_yaw = sensor.odom.yaw
            if curr_yaw is None:
                continue

            error = config.yaw_error(curr_yaw, target)
            if abs(error) < config.TURN_TOLERANCE_DEG:
                break

            direction = 0.5 if error > 0 else -0.5
            executor.exec_gait(2, v_yaw=direction)
            time.sleep(0.05)

    def _leg_odometer(self, ctx: StrategyContext, target_dist: float, target_angle: float) -> None:
        """里程计位移"""
        import math
        executor = ctx.executor
        sensor = ctx.sensor_mgr
        config = ctx.config

        start_xyz = sensor.odom.position
        if start_xyz is None:
            return

        while True:
            curr_xyz = sensor.odom.position
            if curr_xyz is None:
                break

            dist = math.sqrt(
                (curr_xyz[0] - start_xyz[0]) ** 2 +
                (curr_xyz[1] - start_xyz[1]) ** 2
            )
            if dist >= target_dist:
                break

            curr_yaw = sensor.odom.yaw
            if curr_yaw is not None:
                y_err = config.yaw_error(curr_yaw, target_angle)
                executor.exec_gait(7, v_x=config.STRAIGHT_SPEED_M_S, v_yaw=y_err * 0.02)
            else:
                executor.exec_gait(7, v_x=config.STRAIGHT_SPEED_M_S)

            time.sleep(0.02)
