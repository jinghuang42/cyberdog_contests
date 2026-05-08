"""
楼梯策略
"""
import time
from .base_strategy import StageStrategy, StrategyContext


class LoutiStrategy(StageStrategy):
    """楼梯策略"""

    def __init__(self, direction: float):
        super().__init__()
        self._direction = direction
        self._lift_duration = 2.0      # 提拉持续时间
        self._stable_duration = 3.0     # 稳定站立时间
        self._lift_start = 0.0
        self._stable_start = 0.0

    def name(self) -> str:
        return "楼梯"

    def enter(self, ctx: StrategyContext) -> None:
        super().enter(ctx)
        self._lift_start = 0.0
        self._stable_start = 0.0

    def is_complete(self, ctx: StrategyContext) -> bool:
        """楼梯完成判断"""
        return self._phase >= 3

    def execute(self, ctx: StrategyContext) -> None:
        executor = ctx.executor
        sensor = ctx.sensor_mgr
        config = ctx.config

        if self._phase == 0:
            # 阶段0: 爬楼梯前站稳
            executor.act(0, status_monitor=sensor.status)
            self._phase = 1

        elif self._phase == 1:
            # 阶段1: 执行爬楼梯步态
            executor.act(31, status_monitor=sensor.status)
            self._phase = 2
            self._lift_start = time.time()

        elif self._phase == 2:
            # 阶段2: 强制提拉 (防止后腿勾住)
            print(">>> [楼梯] 执行后肢提拉...")
            elapsed = time.time() - self._lift_start
            if elapsed < self._lift_duration:
                # 持续提拉
                executor.exec_gait(32, v_x=config.LIFT_SPEED_M_S,
                                  step_height=config.LIFT_HEIGHT_M, pitch_z=0.08)
            else:
                self._phase = 3
                self._stable_start = time.time()

        elif self._phase == 3:
            # 阶段3: 原地站立稳定
            print(">>> [楼梯] 原地站立稳定...")
            elapsed = time.time() - self._stable_start
            if elapsed < self._stable_duration:
                executor.exec_gait(0, step_height=0.08, pitch_z=0.08)
            else:
                self._phase = 4

        elif self._phase == 4:
            # 阶段4: 完成
            pass
