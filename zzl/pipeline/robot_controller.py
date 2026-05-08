"""
RobotController - 主控制器
负责协调感知、决策、执行各层
"""
import rclpy
import time
import math
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
from typing import Optional, Dict, Type

# 导入传感器
from scan import ROSListener
from y_xyz import LCMListener
from lcm_status import LcmStatus

# 导入配置
from config.stage_config import StageConfig

# 导入状态管理
from state.stage_enum import Stage, TaskId
from state.state_machine import StateMachine, StageTransition

# 导入感知层
from perception.sensor_manager import SensorManager

# 导入控制层
from control.gait_loader import GaitLoader
from control.motion_executor import MotionExecutor

# 导入策略
from strategy.base_strategy import StrategyContext, StageStrategy
from strategy.gaotai_strategy import GaotaiStrategy, ChugaotaiStrategy
from strategy.louti_strategy import LoutiStrategy
from strategy.bridge_strategy import BridgeStrategy
from strategy.wall_strategy import WallSequenceStrategy


class RobotController:
    """
    机器狗主控制器
    职责：初始化、调度各模块、主循环
    """

    def __init__(self, init_angle: float = 0.0):
        """
        Args:
            init_angle: 初始航向角
        """
        self._init_angle = init_angle
        self._heading_ref = init_angle

        # 1. 初始化配置
        self._config = StageConfig()

        # 2. 初始化传感器
        self._ros_listener = ROSListener()
        self._lcm_listener = LCMListener()
        self._lcm_status = LcmStatus()
        self._lcm_status.run()

        self._sensor_mgr = SensorManager(
            self._ros_listener,
            self._lcm_listener,
            self._lcm_status
        )

        # 3. 初始化控制层
        self._gait_loader = GaitLoader(self._config.GAIT_CONFIG_PATH)
        self._executor = MotionExecutor(self._gait_loader)

        # 4. 初始化状态机
        self._state_machine = StateMachine(Stage.IDLE)
        self._stage_transition = StageTransition()

        # 5. 任务状态
        self._task_id = TaskId.CURVE_1
        self._turn_done_xyz = [0.0, 0.0, 0.0]
        self._is_correcting = False
        self._life_count = 0

        # 6. 当前策略
        self._current_strategy: Optional[StageStrategy] = None

        # 7. 策略映射
        self._strategy_map: Dict[Stage, Type[StageStrategy]] = {
            Stage.GAOTAI: GaotaiStrategy,
            Stage.LOUTI: LoutiStrategy,
            Stage.BRIDGE: BridgeStrategy,
            Stage.WALL_HUG: WallSequenceStrategy,
        }

        print(">>> [系统] RobotController 初始化完成")

    def _create_context(self) -> StrategyContext:
        """创建策略上下文"""
        return StrategyContext(
            sensor_mgr=self._sensor_mgr,
            executor=self._executor,
            config=self._config,
            heading_ref=self._heading_ref,
            task_data={}
        )

    def _start_strategy(self, stage: Stage) -> bool:
        """
        启动指定关卡的策略

        Returns:
            是否成功启动
        """
        strategy_class = self._strategy_map.get(stage)
        if strategy_class is None:
            print(f">>> [警告] 未找到 {stage} 对应的策略")
            return False

        # 根据关卡创建策略实例
        if stage == Stage.GAOTAI:
            self._current_strategy = GaotaiStrategy()
        elif stage == Stage.LOUTI:
            self._current_strategy = LoutiStrategy(self._heading_ref)
        elif stage == Stage.BRIDGE:
            self._current_strategy = BridgeStrategy(self._heading_ref)
        elif stage == Stage.WALL_HUG:
            self._current_strategy = WallSequenceStrategy(self._heading_ref)
        else:
            return False

        self._current_strategy.enter(self._create_context())
        return True

    def _execute_strategy(self) -> bool:
        """
        执行当前策略

        Returns:
            策略是否完成
        """
        if self._current_strategy is None:
            return True

        ctx = self._create_context()
        self._current_strategy.execute(ctx)

        if self._current_strategy.is_complete(ctx):
            self._current_strategy.exit(ctx)
            self._current_strategy = None
            return True

        return False

    def _turn_to_angle(self, target_angle: float) -> None:
        """转向到目标角度"""
        config = self._config
        executor = self._executor
        sensor = self._sensor_mgr

        while True:
            curr_yaw = sensor.odom.yaw
            if curr_yaw is None:
                executor.exec_gait(0)
                time.sleep(0.05)
                continue

            error = config.yaw_error(curr_yaw, target_angle)
            if abs(error) < config.TURN_TOLERANCE_DEG:
                break

            direction = config.TURN_SPEED_RAD_S if error > 0 else -config.TURN_SPEED_RAD_S
            executor.exec_gait(2, v_yaw=direction)
            time.sleep(0.05)

        self._heading_ref = target_angle
        self._turn_done_xyz = list(sensor.odom.position or [0, 0, 0])

    def _leg_odometer(self, target_dist: float, target_angle: float) -> None:
        """里程计位移"""
        config = self._config
        executor = self._executor
        sensor = self._sensor_mgr

        start_xyz = sensor.odom.position
        if start_xyz is None:
            return

        while True:
            curr_xyz = sensor.odom.position
            if curr_xyz is None:
                break

            dist = math.sqrt(
                (curr_xyz[0] - start_xyz[0]) ** 2 +
                (curr_xyz[1] - start_xyz[1]) ** 2
            )
            if dist >= target_dist:
                break

            curr_yaw = sensor.odom.yaw
            if curr_yaw is not None:
                y_err = config.yaw_error(curr_yaw, target_angle)
                executor.exec_gait(7, v_x=config.STRAIGHT_SPEED_M_S, v_yaw=y_err * 0.02)
            else:
                executor.exec_gait(7, v_x=config.STRAIGHT_SPEED_M_S)

            time.sleep(0.02)

    def _stage_gaotai(self) -> bool:
        """执行高台关卡"""
        print(">>> [阶段] 高台")
        self._executor.act(29, status_monitor=self._sensor_mgr.status)
        self._executor.act(29, status_monitor=self._sensor_mgr.status)
        self._turn_to_angle(self._heading_ref)
        self._executor.act(0, status_monitor=self._sensor_mgr.status)

        # 上高台
        strategy = GaotaiStrategy()
        strategy.enter(self._create_context())
        while not strategy.is_complete(self._create_context()):
            strategy.execute(self._create_context())
        strategy.exit(self._create_context())

        self._executor.act(29, status_monitor=self._sensor_mgr.status)
        self._executor.act(0, status_monitor=self._sensor_mgr.status)

        # 再上一次
        strategy = GaotaiStrategy()
        strategy.enter(self._create_context())
        while not strategy.is_complete(self._create_context()):
            strategy.execute(self._create_context())
        strategy.exit(self._create_context())

        return True

    def _stage_chugaotai(self) -> bool:
        """出高台"""
        print(">>> [阶段] 出高台")
        self._turn_to_angle(self._heading_ref)

        strategy = ChugaotaiStrategy(self._heading_ref)
        strategy.enter(self._create_context())
        while not strategy.is_complete(self._create_context()):
            strategy.execute(self._create_context())
        strategy.exit(self._create_context())
        return True

    def _stage_louti(self) -> bool:
        """执行楼梯关卡"""
        print(">>> [阶段] 楼梯")
        strategy = LoutiStrategy(self._heading_ref)
        strategy.enter(self._create_context())
        while not strategy.is_complete(self._create_context()):
            strategy.execute(self._create_context())
        strategy.exit(self._create_context())
        return True

    def _stage_bridge(self) -> bool:
        """执行独木桥关卡"""
        print(">>> [阶段] 独木桥")
        strategy = BridgeStrategy(self._heading_ref)
        strategy.enter(self._create_context())
        while not strategy.is_complete(self._create_context()):
            strategy.execute(self._create_context())
        strategy.exit(self._create_context())
        return True

    def _stage_wall_hug(self) -> bool:
        """执行幕布关卡"""
        print(">>> [阶段] 幕布")
        strategy = WallSequenceStrategy(self._heading_ref)
        strategy.enter(self._create_context())
        while not strategy.is_complete(self._create_context()):
            strategy.execute(self._create_context())
        strategy.exit(self._create_context())
        return True

    def _turn_90_left(self) -> None:
        """执行90度左转"""
        print(">>> [转向] 执行90度左转")
        target_yaw = (self._sensor_mgr.odom.yaw + 90) % 360
        self._turn_to_angle(target_yaw)
        self._executor.exec_gait(0, step_height=0.08, pitch_z=0.07)
        time.sleep(0.5)

    def _dash_forward(self, dist: float) -> None:
        """冲刺前进"""
        print(f">>> [冲刺] 前进 {dist}m")
        start_xyz = self._sensor_mgr.odom.position
        if start_xyz is None:
            return

        while True:
            curr_xyz = self._sensor_mgr.odom.position
            if curr_xyz is None:
                continue

            d = math.sqrt(
                (curr_xyz[0] - start_xyz[0]) ** 2 +
                (curr_xyz[1] - start_xyz[1]) ** 2
            )
            if d >= dist:
                break

            scan = self._sensor_mgr.lidar.get_scan()
            curr_yaw = self._sensor_mgr.odom.yaw

            if scan is not None and curr_yaw is not None:
                diff = scan.left - scan.right
                y_err = self._config.yaw_error(curr_yaw, self._heading_ref)
                vy = max(min(diff * 0.2, 0.04), -0.04)
                self._executor.exec_gait(7, v_x=0.15, v_y=vy, v_yaw=y_err * 0.05)
            else:
                self._executor.exec_gait(7, v_x=0.15)

            time.sleep(0.02)

    def _stand_and_lie_down(self) -> None:
        """站稳后趴下"""
        print(">>> [完成] 站稳...")
        self._executor.act(0, status_monitor=self._sensor_mgr.status)
        time.sleep(1.0)

        print(">>> [系统] 趴下")
        from robot_control_cmd_lcmt import robot_control_cmd_lcmt
        import lcm

        lc = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        msg = robot_control_cmd_lcmt()
        msg.mode = 1  # 放松模式
        msg.gait_id = 0

        for _ in range(10):
            msg.life_count = self._life_count % 128
            self._life_count += 1
            lc.publish("robot_control_cmd", msg.encode())
            time.sleep(0.02)

    def run_competition(self) -> None:
        """运行比赛主循环"""
        print(">>> [系统] 等待传感器数据...")

        # 等待传感器就绪
        if not self._sensor_mgr.wait_for_valid(timeout=10.0):
            print(">>> [警告] 传感器数据未就绪，继续运行...")

        # 等待开始信号
        print(">>> [系统] 等待开始信号...")
        while self._sensor_mgr.status.progress == 0:
            self._executor.exec_gait(0)
            time.sleep(0.05)

        print(">>> [系统] 开始比赛!")

        try:
            # ===== 关卡执行序列 =====
            # 1. 高台
            self._stage_gaotai()

            # 2. 出高台
            self._stage_chugaotai()

            # 3. 楼梯
            self._stage_louti()

            # 4. 独木桥
            self._stage_bridge()

            # 5. 90度左转
            self._turn_90_left()

            # 6. 幕布
            self._stage_wall_hug()

            # 7. 终点冲刺
            self._turn_90_left()
            self._dash_forward(1.0)

            # 8. 完成
            self._stand_and_lie_down()

            print(">>> [系统] 比赛完成!")

        except KeyboardInterrupt:
            print(">>> [系统] 被用户中断")
        except Exception as e:
            print(f">>> [错误] 运行时异常: {e}")
            import traceback
            traceback.print_exc()

    def shutdown(self) -> None:
        """关闭控制器"""
        print(">>> [系统] 关闭控制器...")
        self._lcm_status.quit()


def main():
    """主函数"""
    rclpy.init()

    controller = RobotController(init_angle=0)

    # 启动ROS监听线程
    executor = MultiThreadedExecutor()
    executor.add_node(controller._ros_listener)
    Thread(target=executor.spin, daemon=True).start()

    try:
        controller.run_competition()
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
