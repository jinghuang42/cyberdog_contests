"""
雷达传感器封装
"""
import math
from typing import Optional, Tuple
from dataclasses import dataclass


@dataclass
class LidarScan:
    """雷达扫描数据"""
    left: float      # 左侧距离
    front: float     # 正前方距离
    right: float     # 右侧距离
    left_front: float  # 左前方距离
    right_front: float # 右前方距离

    @property
    def center_diff(self) -> float:
        """左右差值 (正值表示偏左)"""
        return self.left - self.right

    def is_finite(self) -> bool:
        """检查数据是否有效"""
        return all(math.isfinite(v) for v in [
            self.left, self.front, self.right,
            self.left_front, self.right_front
        ])


class LidarSensor:
    """雷达传感器封装"""

    def __init__(self, ros_listener):
        """
        Args:
            ros_listener: ROSListener 实例
        """
        self._listener = ros_listener

    def get_scan(self) -> Optional[LidarScan]:
        """
        获取雷达扫描数据

        Returns:
            LidarScan 或 None (数据不可用)
        """
        raw_scan = self._listener.get_scan()
        if raw_scan is None or len(raw_scan) < 5:
            return None

        left, front, right, left_front, right_front = raw_scan

        # 数据清洗
        left = self._clean_value(left)
        front = self._clean_value(front)
        right = self._clean_value(right)
        left_front = self._clean_value(left_front)
        right_front = self._clean_value(right_front)

        return LidarScan(
            left=left,
            front=front,
            right=right,
            left_front=left_front,
            right_front=right_front
        )

    def _clean_value(self, value: float, fallback: float = 1.2) -> float:
        """清洗雷达数据中的无效值"""
        if math.isfinite(value):
            return value
        return fallback

    def get_front_safe(self) -> float:
        """获取前方安全距离 (最小值)"""
        scan = self.get_scan()
        if scan is None:
            return float('inf')
        return min(scan.front, scan.left_front, scan.right_front)
