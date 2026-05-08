"""
状态机实现
"""
from typing import Callable, Dict, List, Optional
from .stage_enum import Stage, Event


class StateMachine:
    """简单状态机"""

    def __init__(self, initial_stage: Stage = Stage.IDLE):
        self._current_stage = initial_stage
        self._stage_history: List[Stage] = []
        self._listeners: Dict[Stage, List[Callable]] = {stage: [] for stage in Stage}

    @property
    def current_stage(self) -> Stage:
        return self._current_stage

    @property
    def stage_history(self) -> List[Stage]:
        return self._listeners.copy()

    def transition(self, new_stage: Stage) -> None:
        """状态转换"""
        if new_stage == self._current_stage:
            return

        old_stage = self._current_stage
        self._stage_history.append(old_stage)
        self._current_stage = new_stage

        # 触发监听器
        for callback in self._listeners[new_stage]:
            callback(old_stage, new_stage)

        print(f">>> [状态机] {old_stage.name} -> {new_stage.name}")

    def on_enter(self, stage: Stage, callback: Callable) -> None:
        """注册进入状态的回调"""
        self._listeners[stage].append(callback)

    def is_stage(self, stage: Stage) -> bool:
        """检查当前状态"""
        return self._current_stage == stage

    def in_stages(self, *stages: Stage) -> bool:
        """检查是否在指定状态中"""
        return self._current_stage in stages


class StageTransition:
    """关卡转换器 - 管理关卡执行顺序"""

    # 线性关卡序列
    STAGE_SEQUENCE = [
        Stage.GAOTAI,
        Stage.LOUTI,
        Stage.BRIDGE,
        Stage.WALL_HUG,
        Stage.DASH,
        Stage.FINISH,
    ]

    def __init__(self):
        self._current_index = 0

    def current(self) -> Stage:
        """获取当前关卡"""
        if self._current_index >= len(self.STAGE_SEQUENCE):
            return Stage.FINISH
        return self.STAGE_SEQUENCE[self._current_index]

    def advance(self) -> Stage:
        """前进到下一个关卡"""
        if self._current_index < len(self.STAGE_SEQUENCE):
            self._current_index += 1
        return self.current()

    def reset(self) -> None:
        """重置到开始"""
        self._current_index = 0

    @property
    def is_complete(self) -> bool:
        """是否完成所有关卡"""
        return self._current_index >= len(self.STAGE_SEQUENCE)

    @property
    def progress(self) -> int:
        """当前进度 (0-100)"""
        if self.is_complete:
            return 100
        return int(self._current_index / len(self.STAGE_SEQUENCE) * 100)
