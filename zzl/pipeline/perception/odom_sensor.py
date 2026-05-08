"""
里程计传感器封装
"""
import math
from typing import Optional, Tuple
from dataclasses import dataclass


@dataclass
class OdomState:
    """里程计状态"""
    x: float          # X坐标
    y: float          # Y坐标
    z: float          # Z坐标 (高度)
    yaw: float         # 航向角 (度)
    xyz: Tuple[float, float, float]  # 位置元组

    @property
    def position_2d(self) -> Tuple[float, float]:
        """二维位置"""
        return (self.x, self.y)

    def distance_from(self, other: 'OdomState') -> float:
        """计算到另一个状态的二维距离"""
        dx = self.x - other.x
        dy = self.y - other.y
        return math.sqrt(dx * dx + dy * dy)


class OdomSensor:
    """里程计传感器封装"""

    def __init__(self, lcm_listener):
        """
        Args:
            lcm_listener: LCMListener 实例
        """
        self._listener = lcm_listener

    def get_state(self) -> Optional[OdomState]:
        """
        获取里程计状态

        Returns:
            OdomState 或 None (数据不可用)
        """
        if self._listener.xyz is None or self._listener.y is None:
            return None

        xyz = self._listener.xyz
        return OdomState(
            x=xyz[0],
            y=xyz[1],
            z=xyz[2],
            yaw=self._listener.y,
            xyz=xyz
        )

    @property
    def position(self) -> Optional[Tuple[float, float, float]]:
        """获取位置 (x, y, z)"""
        return self._listener.xyz

    @property
    def yaw(self) -> Optional[float]:
        """获取航向角"""
        return self._listener.y

    def distance_since(self, start: Tuple[float, float, float]) -> float:
        """
        计算从起始位置出发的距离

        Args:
            start: 起始位置 (x, y, z)

        Returns:
            二维移动距离
        """
        current = self.position
        if current is None:
            return 0.0

        dx = current[0] - start[0]
        dy = current[1] - start[1]
        return math.sqrt(dx * dx + dy * dy)
