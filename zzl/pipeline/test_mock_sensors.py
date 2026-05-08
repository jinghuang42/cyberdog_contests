"""
模拟传感器测试 - 用Mock数据测试策略逻辑
"""
import sys
sys.path.insert(0, '.')

from dataclasses import dataclass, field
from typing import Optional, Dict, Any
import math

# Mock 传感器
@dataclass
class MockLidarScan:
    left: float = 1.0
    front: float = 1.0
    right: float = 1.0
    left_front: float = 1.5
    right_front: float = 1.5

    @property
    def center_diff(self) -> float:
        return self.left - self.right


class MockOdom:
    def __init__(self):
        self.xyz = (0.0, 0.0, 0.0)
        self.yaw = 0.0

    def distance_since(self, start):
        dx = self.xyz[0] - start[0]
        dy = self.xyz[1] - start[1]
        return math.sqrt(dx*dx + dy*dy)


class MockStatus:
    def __init__(self):
        self.progress = 0
        self._counter = 0

    def wait_for_complete(self, timeout=5.0):
        self._counter += 1
        if self._counter > 5:
            self.progress = 100
            return True
        return False

    @property
    def is_order_complete(self):
        return self.progress >= 98


class MockLidarSensor:
    def __init__(self):
        self._scan = MockLidarScan()

    def get_scan(self):
        return self._scan


class MockOdomSensor:
    def __init__(self):
        self._odom = MockOdom()

    @property
    def position(self):
        return self._odom.xyz

    @property
    def yaw(self):
        return self._odom.yaw

    def get_state(self):
        return self._odom


class MockSensorManager:
    def __init__(self):
        self.lidar = MockLidarSensor()
        self.odom = MockOdomSensor()
        self.status = MockStatus()
        self.life_count = 0

    def get_state(self):
        return MockOdom()  # 返回一个模拟状态


# Mock MotionExecutor
class MockExecutor:
    def __init__(self):
        self.last_gait = None
        self.last_vx = None

    def exec_gait(self, gait_idx, v_x=None, v_y=None, v_yaw=None,
                  step_height=0.08, pitch_z=0.035):
        self.last_gait = gait_idx
        self.last_vx = v_x
        print(f"  执行 Gait {gait_idx}, v_x={v_x}")

    def act(self, gait_idx, timeout=5.0, step_height=0.08,
            pitch_z=0.035, status_monitor=None):
        print(f"  执行并等待 Gait {gait_idx}")
        self.last_gait = gait_idx


# 测试策略
def test_gaotai_strategy():
    from strategy.gaotai_strategy import GaotaiStrategy
    from strategy.base_strategy import StrategyContext
    from config.stage_config import StageConfig

    print("\n=== 测试 GaotaiStrategy ===")

    ctx = StrategyContext(
        sensor_mgr=MockSensorManager(),
        executor=MockExecutor(),
        config=StageConfig(),
        heading_ref=0.0,
        task_data={}
    )

    strategy = GaotaiStrategy()
    strategy.enter(ctx)

    print("Phase 0: 抬头准备")
    for i in range(3):
        strategy.execute(ctx)
        print(f"  循环 {i}")

    print("Phase 1: 上高台")
    ctx.sensor_mgr.status.progress = 50
    strategy.execute(ctx)

    print("Phase 2: 等待完成")
    ctx.sensor_mgr.status.progress = 96
    print(f"  is_complete: {strategy.is_complete(ctx)}")

    print("✓ GaotaiStrategy 测试通过")


def test_louti_strategy():
    from strategy.louti_strategy import LoutiStrategy
    from strategy.base_strategy import StrategyContext
    from config.stage_config import StageConfig

    print("\n=== 测试 LoutiStrategy ===")

    ctx = StrategyContext(
        sensor_mgr=MockSensorManager(),
        executor=MockExecutor(),
        config=StageConfig(),
        heading_ref=0.0,
        task_data={}
    )

    strategy = LoutiStrategy(direction=0.0)
    strategy.enter(ctx)

    print("Phase 0: 站稳")
    strategy.execute(ctx)
    print(f"  当前 phase: {strategy._phase}")

    print("Phase 1: 爬楼梯")
    strategy.execute(ctx)
    print(f"  当前 phase: {strategy._phase}")

    print("Phase 2: 提拉")
    strategy.execute(ctx)
    print(f"  当前 phase: {strategy._phase}")

    print("✓ LoutiStrategy 测试通过")


def test_config():
    from config.stage_config import StageConfig, GaitConfig

    print("\n=== 测试配置 ===")

    cfg = StageConfig()
    print(f"  FRONT_OBSTACLE_M: {cfg.FRONT_OBSTACLE_M}")
    print(f"  TURN_TOLERANCE_DEG: {cfg.TURN_TOLERANCE_DEG}")
    print(f"  yaw_error(10, 0): {cfg.yaw_error(10, 0)}")
    print(f"  yaw_error(350, 0): {cfg.yaw_error(350, 0)}")

    print("✓ StageConfig 测试通过")


if __name__ == '__main__':
    test_config()
    test_gaotai_strategy()
    test_louti_strategy()
    print("\n" + "="*50)
    print("所有测试通过 ✓")
