"""
策略基类
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from perception.sensor_manager import SensorManager
    from control.motion_executor import MotionExecutor
    from config.stage_config import StageConfig


@dataclass
class StrategyContext:
    """策略上下文 - 传递各层引用"""
    sensor_mgr: 'SensorManager'
    executor: 'MotionExecutor'
    config: 'StageConfig'
    heading_ref: float = 0.0  # 参考航向
    task_data: dict = None     # 任务相关数据

    def __post_init__(self):
        if self.task_data is None:
            self.task_data = {}


class StageStrategy(ABC):
    """关卡策略基类"""

    def __init__(self):
        self._phase = 0  # 子阶段

    @abstractmethod
    def name(self) -> str:
        """策略名称"""
        pass

    @abstractmethod
    def is_complete(self, ctx: StrategyContext) -> bool:
        """
        判断关卡是否完成

        Args:
            ctx: 策略上下文

        Returns:
            True 表示完成
        """
        pass

    @abstractmethod
    def execute(self, ctx: StrategyContext) -> None:
        """
        执行当前帧的控制指令

        Args:
            ctx: 策略上下文
        """
        pass

    def enter(self, ctx: StrategyContext) -> None:
        """
        进入关卡时的初始化

        Args:
            ctx: 策略上下文
        """
        self._phase = 0
        print(f">>> [策略] 进入 {self.name()}")

    def exit(self, ctx: StrategyContext) -> None:
        """
        退出关卡时的清理

        Args:
            ctx: 策略上下文
        """
        print(f">>> [策略] 退出 {self.name()}")

    def reset(self) -> None:
        """重置策略状态"""
        self._phase = 0


class NavigationStrategy(ABC):
    """导航策略基类 (用于直行、转弯等简单动作)"""

    @abstractmethod
    def name(self) -> str:
        pass

    @abstractmethod
    def execute(self, ctx: StrategyContext) -> bool:
        """
        执行导航

        Returns:
            True 表示完成
        """
        pass
