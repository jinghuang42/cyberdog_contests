"""
动作执行器
"""
import time
import lcm
from typing import Optional
from dataclasses import dataclass
from robot_control_cmd_lcmt import robot_control_cmd_lcmt


@dataclass
class MotionCmd:
    """运动指令"""
    gait_idx: int
    v_x: Optional[float] = None
    v_y: Optional[float] = None
    v_yaw: Optional[float] = None
    step_height: float = 0.08
    pitch_z: float = 0.035
    pitch: float = 0.0  # 俯仰角


class MotionExecutor:
    """动作执行器 - 统一管理运动指令发布"""

    def __init__(self, gait_loader, lcm_url: str = "udpm://239.255.76.67:7671?ttl=255"):
        self._gait = gait_loader
        self._lc = lcm.LCM(lcm_url)
        self._msg = robot_control_cmd_lcmt()
        self._life_count = 0

    def publish(self, cmd: MotionCmd) -> None:
        """
        发布运动指令

        Args:
            cmd: MotionCmd 指令
        """
        step = self._gait.get(cmd.gait_idx)
        if step is None:
            return

        # 填充消息
        self._msg.mode = step["mode"]
        self._msg.gait_id = step["gait_id"]
        self._msg.contact = step["contact"]
        self._msg.value = step["value"]
        self._msg.duration = step["duration"]

        # 速度设置
        vx = cmd.v_x if cmd.v_x is not None else step["vel_des"][0]
        vy = cmd.v_y if cmd.v_y is not None else step["vel_des"][1]
        vyaw = cmd.v_yaw if cmd.v_yaw is not None else step["vel_des"][2]
        self._msg.vel_des = [vx, vy, vyaw]

        # 步高和位置
        self._msg.step_height = [cmd.step_height, cmd.step_height]
        self._msg.pos_des = [0.0, 0.0, cmd.pitch_z]

        # RPY 设置
        self._msg.rpy_des = list(step["rpy_des"])
        if cmd.pitch != 0.0:
            self._msg.rpy_des[1] = cmd.pitch  # 俯仰

        # 其他字段
        self._msg.acc_des = [0.0] * 6
        self._msg.foot_pose = list(step["foot_pose"])
        self._msg.ctrl_point = list(step["ctrl_point"])

        # 发布
        self._msg.life_count = self._life_count % 128
        self._life_count += 1
        self._lc.publish("robot_control_cmd", self._msg.encode())

    def exec_gait(self, gait_idx: int, step_height: float = 0.08,
                  pitch_z: float = 0.035, pitch: float = 0.0) -> None:
        """
        执行指定步态

        Args:
            gait_idx: 步态索引
            step_height: 步高
            pitch_z: 俯仰Z
            pitch: 俯仰角
        """
        cmd = MotionCmd(
            gait_idx=gait_idx,
            step_height=step_height,
            pitch_z=pitch_z,
            pitch=pitch
        )
        self.publish(cmd)

    def act(self, gait_idx: int, timeout: float = 5.0,
            step_height: float = 0.08, pitch_z: float = 0.035,
            status_monitor=None) -> bool:
        """
        执行步态并等待完成

        Args:
            gait_idx: 步态索引
            timeout: 超时时间
            step_height: 步高
            pitch_z: 俯仰Z
            status_monitor: 状态监视器

        Returns:
            是否成功完成
        """
        print(f">>> [执行] Gait {gait_idx}")
        time.sleep(0.1)

        start = time.time()
        rate = 0.02  # 50Hz

        # 等待上一指令完成
        while (time.time() - start) < timeout:
            if status_monitor and status_monitor.progress >= 100:
                break
            self.exec_gait(gait_idx, step_height, pitch_z)
            time.sleep(rate)

        # 等待本指令完成
        start = time.time()
        while (time.time() - start) < timeout:
            if status_monitor and status_monitor.is_order_complete:
                return True
            self.exec_gait(gait_idx, step_height, pitch_z)
            time.sleep(rate)

        return False

    def set_life_count(self, count: int) -> None:
        """设置生命计数"""
        self._life_count = count

    @property
    def life_count(self) -> int:
        return self._life_count
