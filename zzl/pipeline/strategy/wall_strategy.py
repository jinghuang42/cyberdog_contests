"""
幕布策略
"""
import time
import math
from .base_strategy import StageStrategy, StrategyContext


class WallHugStrategy(StageStrategy):
    """幕布/贴墙策略"""

    def __init__(self, side: str, target_dist: float,
                 wall_gap: float = 0.30, stop_at_wall: bool = False,
                 switch_dist: float = None):
        """
        Args:
            side: 贴墙侧 ('left' 或 'right')
            target_dist: 目标距离
            wall_gap: 贴墙距离
            stop_at_wall: 是否在墙前停止
            switch_dist: 切换到雷达居中的距离
        """
        super().__init__()
        self._side = side
        self._target_dist = target_dist
        self._wall_gap = wall_gap
        self._stop_at_wall = stop_at_wall
        self._switch_dist = switch_dist
        self._align_start = 0.0

    def name(self) -> str:
        return f"幕布({self._side})"

    def enter(self, ctx: StrategyContext) -> None:
        super().enter(ctx)
        self._align_start = time.time()

    def is_complete(self, ctx: StrategyContext) -> bool:
        """幕布完成判断"""
        if self._phase >= 2:
            return True

        state = ctx.sensor_mgr.get_state()
        if state.position is None or state.scan is None:
            return False

        start_xyz = ctx.task_data.get('start_xyz', state.position)
        dist_moved = math.sqrt(
            (state.position[0] - start_xyz[0]) ** 2 +
            (state.position[1] - start_xyz[1]) ** 2
        )

        if self._stop_at_wall:
            return dist_moved >= self._target_dist and state.scan.front < ctx.config.WALL_STOP_M
        else:
            return dist_moved >= self._target_dist

    def execute(self, ctx: StrategyContext) -> None:
        executor = ctx.executor
        sensor = ctx.sensor_mgr
        config = ctx.config

        if self._phase == 0:
            # 阶段0: 原地对齐
            self._align_wall(ctx)
            self._phase = 1
            ctx.task_data['start_xyz'] = sensor.odom.position

        elif self._phase == 1:
            # 阶段1: 贴墙前进
            self._wall_hug_advance(ctx)

    def _align_wall(self, ctx: StrategyContext) -> None:
        """贴墙对齐"""
        executor = ctx.executor
        sensor = ctx.sensor_mgr
        config = ctx.config

        print(f">>> [幕布] 向{self._side}平移对齐...")

        while True:
            scan = sensor.lidar.get_scan()
            if scan is None:
                executor.exec_gait(0, step_height=0.08, pitch_z=0.07)
                time.sleep(0.02)
                continue

            if self._side == 'left':
                current_gap = scan.left if math.isfinite(scan.left) else 1.0
            else:
                current_gap = scan.right if math.isfinite(scan.right) else 1.0

            y_err = config.yaw_error(sensor.odom.yaw or 0, ctx.heading_ref)

            # 对齐完成或超时
            if abs(current_gap - self._wall_gap) < config.CENTER_TOLERANCE_M:
                print(f">>> [幕布] 对齐完成，距离: {current_gap:.2f}")
                break

            if time.time() - self._align_start > config.ALIGN_TIMEOUT_S:
                print(f">>> [幕布] 对齐超时，距离: {current_gap:.2f}")
                break

            gap_error = current_gap - self._wall_gap
            if self._side == 'left':
                vy_cmd = gap_error * 0.3
            else:
                vy_cmd = -gap_error * 0.3

            vy_cmd = max(min(vy_cmd, 0.05), -0.05)
            executor.exec_gait(7, v_x=0.0, v_y=vy_cmd, v_yaw=y_err * 0.06,
                              step_height=0.08, pitch_z=0.07)
            time.sleep(0.02)

    def _wall_hug_advance(self, ctx: StrategyContext) -> None:
        """贴墙前进"""
        executor = ctx.executor
        sensor = ctx.sensor_mgr
        config = ctx.config

        print(f">>> [幕布] 贴墙前进...")

        while True:
            state = sensor.get_state()
            if state.position is None or state.yaw is None or state.scan is None:
                executor.exec_gait(0)
                time.sleep(0.02)
                continue

            start_xyz = ctx.task_data.get('start_xyz', state.position)
            dist_moved = math.sqrt(
                (state.position[0] - start_xyz[0]) ** 2 +
                (state.position[1] - start_xyz[1]) ** 2
            )

            scan = state.scan

            # 检查退出条件
            if self._stop_at_wall:
                if dist_moved >= self._target_dist and scan.front < config.WALL_STOP_M:
                    print(f">>> [幕布] 终点探测，距离墙面 {scan.front:.2f}m")
                    break
            else:
                if dist_moved >= self._target_dist:
                    break

            # 计算控制量
            y_err = config.yaw_error(state.yaw, ctx.heading_ref)
            vyaw_cmd = y_err * config.YAW_KP_WALL

            # 模式切换
            if self._switch_dist and dist_moved >= self._switch_dist:
                diff_y = scan.left - scan.right
                vy_cmd = diff_y * 0.15
                mode = "雷达居中"
            else:
                if self._side == 'left':
                    current_gap = scan.left
                else:
                    current_gap = scan.right

                gap_error = current_gap - self._wall_gap
                if self._side == 'left':
                    vy_cmd = gap_error * config.LATERAL_KP_WALL
                else:
                    vy_cmd = -gap_error * config.LATERAL_KP_WALL
                mode = f"{self._side}贴墙"

            # 航向校正
            if abs(y_err) > config.LARGE_YAW_ERR_DEG:
                vx_cmd = 0.07
                vy_cmd = 0.0
            else:
                vx_cmd = config.STRAIGHT_SPEED_M_S

            safe_vy = max(min(vy_cmd, 0.04), -0.04)
            executor.exec_gait(7, v_x=vx_cmd, v_y=safe_vy, v_yaw=vyaw_cmd,
                              step_height=0.08, pitch_z=0.07)

            if sensor.status.progress % 20 == 0:
                print(f">>> [{mode}] 距离:{dist_moved:.2f}m | F:{scan.front:.2f} | 偏角:{y_err:.1f}")

            time.sleep(0.02)

        executor.exec_gait(0)


class WallSequenceStrategy(StageStrategy):
    """幕布序列策略 - 管理多个幕布段"""

    def __init__(self, direction: float):
        super().__init__()
        self._direction = direction
        self._segments = [
            ('right', 1.2, 0.30, False, None),
            ('left', 1.0, 0.30, False, None),
            ('right', 1.1, 0.30, True, 1.1),
        ]
        self._current_segment = 0
        self._current_strategy = None

    def name(self) -> str:
        return "幕布序列"

    def is_complete(self, ctx: StrategyContext) -> bool:
        return self._current_segment >= len(self._segments)

    def execute(self, ctx: StrategyContext) -> None:
        if self._current_segment >= len(self._segments):
            return

        if self._current_strategy is None:
            # 创建当前段策略
            side, dist, gap, stop, switch = self._segments[self._current_segment]
            self._current_strategy = WallHugStrategy(
                side=side,
                target_dist=dist,
                wall_gap=gap,
                stop_at_wall=stop,
                switch_dist=switch
            )
            self._current_strategy.enter(ctx)

        # 执行当前段
        self._current_strategy.execute(ctx)

        # 检查段完成
        if self._current_strategy.is_complete(ctx):
            self._current_strategy.exit(ctx)
            self._current_segment += 1
            self._current_strategy = None
