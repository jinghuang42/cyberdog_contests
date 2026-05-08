"""
配置中心 - 所有魔法数字集中管理
"""
import toml
import os


class StageConfig:
    """所有可调参数集中管理"""

    # ===== 路径配置 =====
    GAIT_CONFIG_PATH = "./toml/usergait.toml"

    # ===== 感知阈值 =====
    FRONT_OBSTACLE_M = 0.23          # 前方障碍停止距离
    WALL_GAP_M = 0.30               # 贴墙安全距离
    CENTER_TOLERANCE_M = 0.05       # 居中容差
    BRIDGE_EXIT_M = 3.5             # 独木桥雷达矫正距离
    WALL_STOP_M = 0.30              # 幕布停止距离
    LIFT_HEIGHT_M = 0.13            # 提拉步高
    STAIR_LIFT_DIST_M = 0.18        # 楼梯提拉速度(m/s)

    # ===== 速度参数 =====
    STRAIGHT_SPEED_M_S = 0.20       # 直线速度
    SLOW_SPEED_M_S = 0.12          # 慢速（障碍前）
    TURN_SPEED_RAD_S = 0.50        # 转弯速度
    LIFT_SPEED_M_S = 0.18          # 提拉速度
    CORRECTION_SPEED_M_S = 0.05     # 矫正速度
    LATERAL_SPEED_M_S = 0.08       # 横向移动速度

    # ===== 角度参数 =====
    TURN_TOLERANCE_DEG = 4.0        # 转弯精度
    YAW_LOCK_DEG = 1.5             # 偏航锁定阈值
    YAW_RECOVERY_DEG = 1.0         # 姿态恢复阈值
    LARGE_YAW_ERR_DEG = 1.5        # 大角度误差阈值

    # ===== 时间参数 =====
    LIFT_DURATION_S = 2.0           # 提拉持续时间
    STABLE_DURATION_S = 3.0        # 稳定站立时间
    ALIGN_TIMEOUT_S = 3.0           # 对齐超时时间

    # ===== PID参数 =====
    LATERAL_KP = 0.35              # 横向P系数
    YAW_KP = 0.10                  # 航向P系数
    LATERAL_KP_WALL = 0.30         # 贴墙横向P系数
    YAW_KP_WALL = 0.06             # 贴墙航向P系数
    LATERAL_INTEGRAL_MAX = 0.5     # 积分上限
    YAW_INTEGRAL_MAX = 0.5         # 积分上限

    # ===== 步态参数 =====
    DEFAULT_STEP_HEIGHT = 0.08     # 默认步高
    DEFAULT_PITCH_Z = 0.035        # 默认俯仰
    HIGH_STEP_HEIGHT = 0.10        # 高抬腿步高
    GAOTAI_STEP_HEIGHT = 0.23      # 高台步高

    # ===== 距离参数 =====
    GAOTAI_APPROACH_DIST = 0.20    # 高台接近距离
    TURNTABLE_DIST = 0.40          # 转弯里程计距离
    DASH_DIST = 1.0                # 冲刺距离

    # ===== 幕布参数 =====
    WALL_HUG_DIST_1 = 1.2          # 幕布第一段距离
    WALL_HUG_DIST_2 = 1.0          # 幕布第二段距离
    WALL_HUG_DIST_3 = 1.1          # 幕布第三段距离
    WALL_SWITCH_DIST = 1.1         # 幕布切换距离

    # ===== 雷达数据清洗 =====
    LIDAR_VALID_MIN = 0.05         # 雷达最小有效距离
    LIDAR_VALID_MAX = 10.0         # 雷达最大有效距离
    LIDAR_INF_FALLBACK = 1.2       # 雷达无穷远默认值

    # ===== 进度阈值 =====
    GAOTAI_PROGRESS = 95            # 高台完成进度
    ORDER_COMPLETE_THRESHOLD = 98  # 指令完成阈值

    # ===== 偏航计算 =====
    @staticmethod
    def normalize_yaw(yaw):
        """将角度归一化到 [0, 360)"""
        return yaw % 360

    @staticmethod
    def yaw_error(current, target):
        """计算航向误差 (target - current)，归一化到 (-180, 180]"""
        return (target - current + 180) % 360 - 180


class GaitConfig:
    """步态配置加载器"""

    _instance = None
    _steps = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._load_config()
        return cls._instance

    def _load_config(self):
        """加载步态配置文件"""
        try:
            self._steps = toml.load(self.GAIT_CONFIG_PATH)
            print(f">>> [配置] 步态配置加载成功: {self.GAIT_CONFIG_PATH}")
        except Exception as e:
            print(f">>> [错误] 步态配置加载失败: {e}")
            self._steps = {"step": []}

    def get_step(self, gait_idx):
        """获取指定步态的配置"""
        if self._steps is None:
            return None
        try:
            return self._steps["step"][int(gait_idx)]
        except (IndexError, ValueError, TypeError):
            return None

    def reload(self):
        """重新加载配置"""
        self._load_config()
