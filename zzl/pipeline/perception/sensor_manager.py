"""
传感器管理器 - 统一管理所有传感器
"""
from typing import Optional
from dataclasses import dataclass
from .lidar_sensor import LidarSensor, LidarScan
from .odom_sensor import OdomSensor, OdomState


@dataclass
class RobotState:
    """机器人统一状态"""
    position: Optional[tuple] = None      # (x, y, z)
    yaw: Optional[float] = None           # 航向角
    scan: Optional[LidarScan] = None      # 雷达扫描
    progress: int = 0                     # 任务进度


class SensorManager:
    """传感器统一管理器"""

    def __init__(self, ros_listener, lcm_listener, lcm_status):
        """
        Args:
            ros_listener: ROSListener 实例 (雷达)
            lcm_listener: LCMListener 实例 (里程计)
            lcm_status: LcmStatus 实例 (状态)
        """
        self.lidar = LidarSensor(ros_listener)
        self.odom = OdomSensor(lcm_listener)
        self.status = StatusMonitor(lcm_status)

    def get_state(self) -> RobotState:
        """
        获取机器人统一状态

        Returns:
            RobotState
        """
        return RobotState(
            position=self.odom.position,
            yaw=self.odom.yaw,
            scan=self.lidar.get_scan(),
            progress=self.status.progress
        )

    def wait_for_valid(self, timeout: float = 5.0) -> bool:
        """
        等待获取有效的传感器数据

        Args:
            timeout: 超时时间(秒)

        Returns:
            是否成功获取
        """
        import time
        start = time.time()

        while (time.time() - start) < timeout:
            state = self.get_state()
            if state.position is not None and state.yaw is not None:
                return True

        return False


class StatusMonitor:
    """状态监控器"""

    def __init__(self, lcm_status):
        self._status = lcm_status

    @property
    def progress(self) -> int:
        """获取任务进度"""
        try:
            return self._status.rec_msg.order_process_bar
        except AttributeError:
            return 0

    @property
    def is_order_complete(self) -> bool:
        """当前指令是否完成"""
        return self.progress >= 98

    def wait_for_complete(self, timeout: float = 5.0) -> bool:
        """等待指令完成"""
        import time
        start = time.time()

        while (time.time() - start) < timeout:
            if self.is_order_complete:
                return True

        return False
