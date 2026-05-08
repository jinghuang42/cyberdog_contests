"""
状态枚举定义
"""
from enum import Enum, auto


class Stage(Enum):
    """主舞台枚举"""
    IDLE = auto()           # 空闲/等待启动
    WAIT_START = auto()     # 等待开始信号
    STRAIGHT = auto()       # 直行
    TURN = auto()           # 转弯
    GAOTAI = auto()         # 高台
    LOUTI = auto()          # 楼梯
    BRIDGE = auto()         # 独木桥
    WALL_HUG = auto()       # 幕布
    DASH = auto()           # 冲刺
    FINISH = auto()         # 完成


class Event(Enum):
    """事件枚举"""
    START_RECEIVED = auto()        # 收到开始信号
    OBSTACLE_AHEAD = auto()        # 前方有障碍
    WALL_DETECTED = auto()         # 检测到墙
    TURN_NEEDED = auto()           # 需要转弯
    STAGE_COMPLETE = auto()        # 关卡完成
    ALIGNMENT_OK = auto()          # 对齐完成
    TIME_TIMEOUT = auto()          # 超时


class TaskId(Enum):
    """任务ID枚举 (替代原来的 task_id)"""
    CURVE_1 = 0        # 弯道1
    TRANSITION = 1    # 过渡
    CYLINDER = 2      # 圆柱障碍
    CURVE_2 = 3       # 弯道2
    GAOTAI_APPROACH = 4  # 高台对齐


class SubStage(Enum):
    """子阶段枚举"""
    NONE = auto()
    APPROACH = auto()      # 接近
    EXECUTE = auto()       # 执行
    RECOVERY = auto()      # 恢复
    ALIGN = auto()          # 对齐
    LIFT = auto()           # 提拉
    STABILIZE = auto()     # 稳定


class Direction(Enum):
    """方向枚举"""
    LEFT = 'left'
    RIGHT = 'right'
    FRONT = 'front'
    BACK = 'back'
