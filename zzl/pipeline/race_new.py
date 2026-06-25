import math
import sys
import time
from enum import IntEnum
from threading import Thread

import lcm
import rclpy
import toml
from rclpy.executors import MultiThreadedExecutor

from lcm_status import LcmStatus
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from scan import ROSListener
from y_xyz import LCMListener


class RaceStage(IntEnum):
    WAIT_START = 0
    CORRIDOR = 1
    TURN_LEFT = 2
    PILLAR_ARC = 3
    LEAVE_PILLAR = 5
    OBSTACLE_CHAIN = 6
    FINISHED = 100


class TaskId(IntEnum):
    FIRST_TURN = 0
    TRANSITION_TO_PILLAR = 1
    PILLAR = 2
    SECOND_TURN = 3
    OBSTACLE_CHAIN = 4
    DONE = 100


class RaceConfig:
    GAIT_CONFIG_PATH = "./toml/usergait.toml"
    CONTROL_LCM_URL = "udpm://239.255.76.67:7671?ttl=255"

    CONTROL_DT = 0.02
    SENSOR_WAIT_DT = 0.05

    DEFAULT_HEIGHT = 0.08
    DEFAULT_POS_Z = 0.035
    LOW_POS_Z = 0.07

    CORRIDOR_FAST_VX = 0.20
    CORRIDOR_SLOW_VX = 0.12
    CORRIDOR_FRONT_SLOW_M = 0.60
    CORRIDOR_TURN_FRONT_M = 0.23
    CORRIDOR_CENTER_START_M = 0.18
    CORRIDOR_CENTER_STOP_M = 0.08

    TURN_TOL_DEG = 3.0
    STRONG_TURN_TOL_DEG = 4.0
    TURN_SPEED = 0.50

    PILLAR_FRONT_M = 0.30
    PILLAR_SIDE_M = 1.00
    PILLAR_LEAVE_DIST_M = 1.50

    BRIDGE_DIST_M = 3.50
    BRIDGE_EXIT_FRONT_M = 0.40

    WALL_GAP_M = 0.30


class SensorSnapshot:
    def __init__(self, scan, yaw, xyz):
        self.scan = scan
        self.yaw = yaw
        self.xyz = xyz
        self.left = scan[0]
        self.front = scan[1]
        self.right = scan[2]
        self.left_front = scan[3]
        self.right_front = scan[4]


class RaceNewMaster:
    """
    Structured rewrite of yunsheng333.py.

    The behavior is intentionally close to the working script, but the code is
    split into clear layers: state, sensors, motion primitives, obstacle stages,
    and the main state machine.
    """

    def __init__(self, init_angle=0.0, config=None):
        self.config = config or RaceConfig()

        self.lidar = ROSListener()
        self.odom = LCMListener()
        self.status = LcmStatus()
        self.ctrl_lc = lcm.LCM(self.config.CONTROL_LCM_URL)
        self.msg = robot_control_cmd_lcmt()

        self.init_angle = init_angle
        self.stage = RaceStage.WAIT_START
        self.task_id = TaskId.FIRST_TURN
        self.sub_stage = 0
        self.sub_stage_start_time = 0.0

        self.life_count = 0
        self.is_correcting = False
        self.turn_done_xyz = [0.0, 0.0, 0.0]
        self.last_log_time = 0.0

        self.steps = self._load_gait_config(self.config.GAIT_CONFIG_PATH)
        self.status.run()

    # ---------------------------------------------------------------------
    # Small utilities
    # ---------------------------------------------------------------------
    def _load_gait_config(self, path):
        try:
            steps = toml.load(path)
            print(">>> [config] gait config loaded: {}".format(path))
            return steps
        except Exception as exc:
            print(">>> [error] failed to load gait config {}: {}".format(path, exc))
            sys.exit(1)

    @staticmethod
    def clamp(value, low, high):
        return max(min(value, high), low)

    @staticmethod
    def is_finite(value):
        return value is not None and math.isfinite(value)

    @staticmethod
    def yaw_error(target, current):
        return (target - current + 180.0) % 360.0 - 180.0

    @staticmethod
    def distance_2d(a, b):
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def _progress(self):
        try:
            return self.status.rec_msg.order_process_bar
        except AttributeError:
            return 0

    def _read_sensors(self):
        scan = self.lidar.get_scan()
        yaw = self.odom.y
        xyz = self.odom.xyz
        if scan is None or yaw is None or xyz is None or len(scan) < 5:
            return None
        return SensorSnapshot(scan, yaw, xyz)

    def _sleep(self, seconds=None):
        time.sleep(self.config.CONTROL_DT if seconds is None else seconds)

    def _stand_heartbeat(self, height=None, pos_z=None):
        self.publish_gait(
            0,
            height=self.config.DEFAULT_HEIGHT if height is None else height,
            pos_z=self.config.DEFAULT_POS_Z if pos_z is None else pos_z,
        )

    # ---------------------------------------------------------------------
    # Motion command layer
    # ---------------------------------------------------------------------
    def publish_gait(
        self,
        gait_idx,
        v_x=None,
        v_y=None,
        v_yaw=None,
        height=None,
        pos_z=None,
        pitch=None,
    ):
        try:
            step = self.steps["step"][int(gait_idx)]
        except Exception:
            return False

        height = self.config.DEFAULT_HEIGHT if height is None else height
        pos_z = self.config.DEFAULT_POS_Z if pos_z is None else pos_z

        vx = step["vel_des"][0] if v_x is None else v_x
        vy = step["vel_des"][1] if v_y is None else v_y
        vyaw = step["vel_des"][2] if v_yaw is None else v_yaw

        self.msg.mode = step["mode"]
        self.msg.gait_id = step["gait_id"]
        self.msg.contact = step["contact"]
        self.msg.value = step["value"]
        self.msg.duration = step["duration"]
        self.msg.vel_des = [vx, vy, vyaw]
        self.msg.step_height = [height, height]
        self.msg.pos_des = [0.0, 0.0, pos_z]
        self.msg.acc_des = [0.0] * 6
        self.msg.foot_pose = [0.0] * 6
        self.msg.ctrl_point = [0.0] * 6

        for i in range(3):
            self.msg.rpy_des[i] = step["rpy_des"][i]
            self.msg.foot_pose[i] = step["foot_pose"][i]
            self.msg.ctrl_point[i] = step["ctrl_point"][i]

        if pitch is not None:
            self.msg.rpy_des[1] = pitch

        self.msg.life_count = self.life_count % 128
        self.life_count += 1
        self.ctrl_lc.publish("robot_control_cmd", self.msg.encode())
        return True

    def act(self, gait_idx, height=None, pos_z=None):
        print(">>> [act] gait {}".format(gait_idx))
        time.sleep(0.1)

        while self._progress() >= 100:
            self.publish_gait(gait_idx, height=height, pos_z=pos_z)
            self._sleep()

        while self._progress() < 98:
            self.publish_gait(gait_idx, height=height, pos_z=pos_z)
            self._sleep()

    def relax(self):
        for _ in range(10):
            self.msg.mode = 1
            self.msg.gait_id = 0
            self.msg.life_count = self.life_count % 128
            self.life_count += 1
            self.ctrl_lc.publish("robot_control_cmd", self.msg.encode())
            self._sleep()

    # ---------------------------------------------------------------------
    # Basic movement primitives
    # ---------------------------------------------------------------------
    def turn_step(self, target_angle, tolerance=None, yaw_speed=None):
        tolerance = self.config.TURN_TOL_DEG if tolerance is None else tolerance
        yaw_speed = self.config.TURN_SPEED if yaw_speed is None else yaw_speed

        if self.odom.y is None:
            self._stand_heartbeat()
            return True

        error = self.yaw_error(target_angle, self.odom.y)
        if abs(error) < tolerance:
            return False

        self.publish_gait(2, v_yaw=yaw_speed if error > 0 else -yaw_speed)
        self._sleep(0.05)
        return True

    def turn_to(self, target_angle, tolerance=None, yaw_speed=None):
        while self.turn_step(target_angle, tolerance=tolerance, yaw_speed=yaw_speed):
            pass

    def move_by_odometer(self, target_dist, target_angle, v_x=0.15):
        print(">>> [odom] move {:.2f}m".format(target_dist))

        while self.odom.xyz is None:
            self._stand_heartbeat()
            self._sleep()

        start_xyz = list(self.odom.xyz)
        while True:
            curr_xyz = self.odom.xyz
            curr_yaw = self.odom.y
            if curr_xyz is None or curr_yaw is None:
                self._stand_heartbeat()
                self._sleep()
                continue

            if self.distance_2d(curr_xyz, start_xyz) >= target_dist:
                break

            yaw_err = self.yaw_error(target_angle, curr_yaw)
            self.publish_gait(7, v_x=v_x, v_yaw=yaw_err * 0.02)
            self._sleep()

    def center_by_lidar(self, tolerance=0.05):
        while True:
            snapshot = self._read_sensors()
            if snapshot is None:
                self._stand_heartbeat()
                self._sleep()
                continue

            diff = snapshot.left - snapshot.right
            if abs(diff) < tolerance:
                break

            vy_cmd = 0.08 if diff > 0 else -0.08
            self.publish_gait(7, v_x=0.0, v_y=vy_cmd, v_yaw=0.0)
            self._sleep()

    def centered_forward(self, target_dist, target_angle, v_x=0.15):
        start_xyz = None
        while True:
            snapshot = self._read_sensors()
            if snapshot is None:
                self._stand_heartbeat()
                self._sleep()
                continue

            if start_xyz is None:
                start_xyz = list(snapshot.xyz)
                continue

            if self.distance_2d(snapshot.xyz, start_xyz) >= target_dist:
                break

            diff_y = snapshot.left - snapshot.right
            if diff_y > 0.03:
                vy_cmd = 0.08
            elif diff_y < -0.03:
                vy_cmd = -0.08
            else:
                vy_cmd = 0.0

            yaw_err = self.yaw_error(target_angle, snapshot.yaw)
            self.publish_gait(7, v_x=v_x, v_y=vy_cmd, v_yaw=yaw_err * 0.02)
            self._sleep()

        self._stand_heartbeat()

    # ---------------------------------------------------------------------
    # Obstacle stages
    # ---------------------------------------------------------------------
    def pass_high_platform_once(self):
        print(">>> [stage] high platform")
        for _ in range(30):
            self.publish_gait(0, height=0.08, pos_z=0.10, pitch=0.30)
            self._sleep()

        self.act(22, height=0.23, pos_z=0.10)
        self.act(24)
        self.act(0)
        self.act(29)
        self.act(0)

    def pass_stairs(self, at_direction):
        print(">>> [stage] stairs")
        self.act(0)
        self.act(31)

        for _ in range(100):
            self.publish_gait(32, v_x=0.18, height=0.13, pos_z=0.08)
            self._sleep()

        for i in range(150):
            self.publish_gait(0, height=0.08, pos_z=0.08)
            if i % 50 == 0:
                print(">>> [stairs] stable {:.1f}s".format(i * self.config.CONTROL_DT))
            self._sleep()

    def exit_high_platform(self, at_direction):
        print(">>> [stage] exit high platform")
        self.act(25)
        self.act(0)

        self.center_by_lidar(tolerance=0.05)
        self._stand_heartbeat()
        time.sleep(0.2)

        self.turn_to(at_direction)
        self.move_by_odometer(0.02, at_direction)

        self.act(25)
        self.act(0)
        self.turn_to(at_direction)
        self.centered_forward(0.54, at_direction, v_x=0.15)

    def pass_bridge(self, at_direction=None):
        if at_direction is None:
            at_direction = self.odom.y

        print(">>> [stage] bridge, heading {:.2f}".format(at_direction))

        for _ in range(60):
            curr_yaw = self.odom.y
            if curr_yaw is not None:
                yaw_err = self.yaw_error(at_direction, curr_yaw)
                self.publish_gait(7, v_x=0.0, v_y=0.0, v_yaw=yaw_err * 0.08, height=0.08, pos_z=0.07)
            self._sleep()

        start_xyz = None
        bridge_exit_mode = False

        while True:
            snapshot = self._read_sensors()
            if snapshot is None:
                self.publish_gait(0, height=0.08, pos_z=0.07)
                self._sleep()
                continue

            if start_xyz is None:
                start_xyz = list(snapshot.xyz)
                continue

            dist_moved = self.distance_2d(snapshot.xyz, start_xyz)
            left = snapshot.left if self.is_finite(snapshot.left) else 1.2
            right = snapshot.right if self.is_finite(snapshot.right) else 1.2
            left_front = snapshot.left_front if self.is_finite(snapshot.left_front) else 1.5
            right_front = snapshot.right_front if self.is_finite(snapshot.right_front) else 1.5
            diff = left - right
            yaw_err = self.yaw_error(at_direction, snapshot.yaw)

            vyaw_cmd = yaw_err * 0.10
            if abs(yaw_err) > 1.0:
                vx_cmd = 0.05
                vy_cmd = 0.0
                status_note = "yaw-correction"
            else:
                vx_cmd = 0.10
                vy_cmd = self.clamp(diff * 0.10, -0.03, 0.03)
                status_note = "center-forward"

            if dist_moved >= self.config.BRIDGE_DIST_M:
                if not bridge_exit_mode:
                    print(">>> [bridge] exit correction")
                    for _ in range(50):
                        scan = self.lidar.get_scan()
                        curr_yaw = self.odom.y
                        if scan is None or curr_yaw is None:
                            continue

                        lidar_err_raw = (scan[3] - scan[0]) - (scan[4] - scan[2])
                        lidar_err = lidar_err_raw if math.isfinite(lidar_err_raw) else 0.0
                        odom_err = self.yaw_error(at_direction, curr_yaw)
                        total_yaw_cmd = odom_err * 0.08 + lidar_err * 0.5
                        self.publish_gait(7, v_x=0.0, v_y=0.0, v_yaw=total_yaw_cmd, height=0.08, pos_z=0.07)
                        self._sleep()
                    bridge_exit_mode = True

                height = 0.10
                pos_z = 0.07
                if snapshot.front < self.config.BRIDGE_EXIT_FRONT_M:
                    break
            else:
                height = 0.08
                pos_z = 0.07

            self.publish_gait(7, v_x=vx_cmd, v_y=vy_cmd, v_yaw=vyaw_cmd, height=height, pos_z=pos_z)
            if self.life_count % 15 == 0:
                print(
                    "[{}] dist:{:.2f}m | L-R:{:.2f} | yaw:{:.1f} | vy:{:.3f}".format(
                        status_note, dist_moved, diff, yaw_err, vy_cmd
                    )
                )
            self._sleep()

        self._stand_heartbeat()
        print(">>> [stage] bridge done")

    def wall_hug_advance(self, side, target_dist, wall_gap=None, stop_at_wall=False, switch_dist=None):
        wall_gap = self.config.WALL_GAP_M if wall_gap is None else wall_gap

        for _ in range(35):
            self.publish_gait(0, height=0.08, pos_z=0.07)
            self._sleep()

        print(">>> [wall] align side={} gap={:.2f}".format(side, wall_gap))
        align_start = time.time()
        while True:
            snapshot = self._read_sensors()
            if snapshot is None:
                self.publish_gait(0, height=0.08, pos_z=0.07)
                self._sleep()
                continue

            raw_gap = snapshot.left if side == "left" else snapshot.right
            current_gap = raw_gap if self.is_finite(raw_gap) else 1.0
            yaw_err = self.yaw_error(self.init_angle, snapshot.yaw)

            if abs(current_gap - wall_gap) < 0.05 or (time.time() - align_start > 3.0):
                print(">>> [wall] aligned gap={:.2f}".format(current_gap))
                break

            if side == "left":
                vy_cmd = (current_gap - wall_gap) * 0.3
            else:
                vy_cmd = -(current_gap - wall_gap) * 0.3

            self.publish_gait(
                7,
                v_x=0.0,
                v_y=self.clamp(vy_cmd, -0.05, 0.05),
                v_yaw=yaw_err * 0.06,
                height=0.08,
                pos_z=0.07,
            )
            self._sleep()

        print(">>> [wall] advance")
        start_xyz = list(self.odom.xyz)
        while True:
            snapshot = self._read_sensors()
            if snapshot is None:
                self.publish_gait(0, height=0.08, pos_z=0.07)
                self._sleep()
                continue

            dist_moved = self.distance_2d(snapshot.xyz, start_xyz)
            left = snapshot.left if self.is_finite(snapshot.left) else 1.2
            right = snapshot.right if self.is_finite(snapshot.right) else 1.2

            if stop_at_wall:
                if dist_moved >= target_dist and snapshot.front < 0.30:
                    print(">>> [wall] stop at front wall {:.2f}m".format(snapshot.front))
                    break
            elif dist_moved >= target_dist:
                break

            if switch_dist and dist_moved >= switch_dist:
                vy_cmd = (left - right) * 0.15
                mode_label = "center-by-lidar"
            else:
                current_gap = left if side == "left" else right
                if side == "left":
                    vy_cmd = (current_gap - wall_gap) * 0.35
                else:
                    vy_cmd = -(current_gap - wall_gap) * 0.35
                mode_label = "{}-wall".format(side)

            yaw_err = self.yaw_error(self.init_angle, snapshot.yaw)
            if abs(yaw_err) > 1.5:
                vx_cmd = 0.07
                vy_cmd = 0.0
            else:
                vx_cmd = 0.15

            self.publish_gait(
                7,
                v_x=vx_cmd,
                v_y=self.clamp(vy_cmd, -0.04, 0.04),
                v_yaw=yaw_err * 0.06,
                height=0.08,
                pos_z=0.07,
            )
            if self.life_count % 20 == 0:
                print(">>> [{}] dist:{:.2f}m | F:{:.2f} | yaw:{:.1f}".format(mode_label, dist_moved, snapshot.front, yaw_err))
            self._sleep()

        self._stand_heartbeat()

    def dash_centered(self, target_dist, v_x=0.15):
        print(">>> [finish] centered dash {:.2f}m".format(target_dist))
        start_xyz = list(self.odom.xyz)
        while True:
            snapshot = self._read_sensors()
            if snapshot is None:
                self._stand_heartbeat(height=0.08, pos_z=0.07)
                self._sleep()
                continue

            if self.distance_2d(snapshot.xyz, start_xyz) >= target_dist:
                break

            diff = snapshot.left - snapshot.right
            yaw_err = self.yaw_error(self.init_angle, snapshot.yaw)
            self.publish_gait(
                7,
                v_x=v_x,
                v_y=self.clamp(diff * 0.2, -0.04, 0.04),
                v_yaw=yaw_err * 0.05,
            )
            self._sleep()

    # ---------------------------------------------------------------------
    # Main race state machine
    # ---------------------------------------------------------------------
    def run_competition(self):
        while self.stage != RaceStage.FINISHED:
            snapshot = self._read_sensors()
            if snapshot is None:
                self._stand_heartbeat()
                self._sleep(self.config.SENSOR_WAIT_DT)
                continue

            if self.stage == RaceStage.WAIT_START:
                self._handle_wait_start()
            elif self.stage == RaceStage.CORRIDOR:
                self._handle_corridor(snapshot)
            elif self.stage == RaceStage.TURN_LEFT:
                self._handle_turn_left(snapshot)
            elif self.stage == RaceStage.PILLAR_ARC:
                self._handle_pillar_arc(snapshot)
            elif self.stage == RaceStage.LEAVE_PILLAR:
                self._handle_leave_pillar(snapshot)
            elif self.stage == RaceStage.OBSTACLE_CHAIN:
                self._run_obstacle_chain()
            else:
                self.stage = RaceStage.FINISHED

        print(">>> [race] finished")

    def _handle_wait_start(self):
        self._stand_heartbeat()
        if self._progress() > 0:
            self.stage = RaceStage.CORRIDOR

    def _handle_corridor(self, snapshot):
        dist_from_turn = self.distance_2d(snapshot.xyz, self.turn_done_xyz)

        if self.task_id in (TaskId.FIRST_TURN, TaskId.SECOND_TURN) and snapshot.front < self.config.CORRIDOR_TURN_FRONT_M:
            self.stage = RaceStage.TURN_LEFT
        elif self.task_id == TaskId.PILLAR:
            if snapshot.front < self.config.PILLAR_FRONT_M and (
                snapshot.left_front < self.config.PILLAR_SIDE_M or snapshot.left < self.config.PILLAR_SIDE_M
            ):
                self.stage = RaceStage.PILLAR_ARC
                self.sub_stage = 1
                self.sub_stage_start_time = time.time()
        elif self.task_id == TaskId.OBSTACLE_CHAIN:
            self.stage = RaceStage.OBSTACLE_CHAIN

        if self.task_id == TaskId.TRANSITION_TO_PILLAR and dist_from_turn > 0.4:
            self.task_id = TaskId.PILLAR

        if self.task_id == TaskId.OBSTACLE_CHAIN:
            return

        vx_target = self.config.CORRIDOR_FAST_VX if snapshot.front > self.config.CORRIDOR_FRONT_SLOW_M else self.config.CORRIDOR_SLOW_VX
        diff = snapshot.left - snapshot.right

        if self.is_correcting:
            lateral_gait = 5 if diff > 0 else 6
            lateral_vy = 0.06 if diff > 0 else -0.06
            self.publish_gait(lateral_gait, v_x=0.15, v_y=lateral_vy)
            if abs(diff) < self.config.CORRIDOR_CENTER_STOP_M:
                self.is_correcting = False
        else:
            self.publish_gait(7, v_x=vx_target)
            if abs(diff) > self.config.CORRIDOR_CENTER_START_M:
                self.is_correcting = True

    def _handle_turn_left(self, snapshot):
        target_yaw = (snapshot.yaw + 90.0) % 360.0
        self.turn_to(target_yaw, tolerance=self.config.STRONG_TURN_TOL_DEG, yaw_speed=0.5)

        self.init_angle = target_yaw
        self.turn_done_xyz = list(self.odom.xyz)
        if self.task_id == TaskId.FIRST_TURN:
            self.task_id = TaskId.TRANSITION_TO_PILLAR
        else:
            self.task_id = TaskId.OBSTACLE_CHAIN
        self.stage = RaceStage.CORRIDOR

    def _handle_pillar_arc(self, snapshot):
        yaw_err = self.yaw_error(self.init_angle, snapshot.yaw)

        if self.sub_stage == 1:
            self.publish_gait(7, v_x=0.10, v_y=-0.45, v_yaw=-0.55)
            if time.time() - self.sub_stage_start_time > 2.2:
                self.sub_stage = 2
        elif self.sub_stage == 2:
            yaw_cmd = 0.70 + (snapshot.left - 0.65) * 1.5
            self.publish_gait(7, v_x=0.22, v_yaw=yaw_cmd)
            if snapshot.right_front > 1.8 or snapshot.right > 1.8:
                self.sub_stage = 3
                self.sub_stage_start_time = time.time()
        elif self.sub_stage == 3:
            if abs(yaw_err) > 8.0:
                self.publish_gait(7, v_x=0.12, v_yaw=-0.6)
            elif time.time() - self.sub_stage_start_time < 1.5:
                self.publish_gait(7, v_x=0.20)
            else:
                self.turn_done_xyz = list(self.odom.xyz)
                self.sub_stage = 0
                self.stage = RaceStage.LEAVE_PILLAR

    def _handle_leave_pillar(self, snapshot):
        dist_from_turn = self.distance_2d(snapshot.xyz, self.turn_done_xyz)
        if dist_from_turn < self.config.PILLAR_LEAVE_DIST_M:
            self.publish_gait(7, v_x=0.25)
        else:
            self.task_id = TaskId.SECOND_TURN
            self.stage = RaceStage.CORRIDOR

    def _run_obstacle_chain(self):
        at_direction = self.init_angle

        print(">>> [chain] high platform preparation")
        self.move_by_odometer(0.20, at_direction)
        self.act(29)
        self.act(29)
        self.turn_to(at_direction)
        self.act(0)

        self.pass_high_platform_once()
        self.act(29)
        self.act(0)
        self.pass_high_platform_once()

        self.turn_to(at_direction)
        self.exit_high_platform(at_direction)

        self.pass_stairs(at_direction)
        self.pass_bridge(at_direction)

        print(">>> [chain] turn left after bridge")
        target_yaw = (self.odom.y + 90.0) % 360.0
        self.turn_to(target_yaw, tolerance=4.0, yaw_speed=0.5)
        self.init_angle = target_yaw
        self.turn_done_xyz = list(self.odom.xyz)
        self.publish_gait(0, height=0.08, pos_z=0.07)
        time.sleep(0.5)

        self.wall_hug_advance(side="right", target_dist=1.2, wall_gap=self.config.WALL_GAP_M)
        self.wall_hug_advance(side="left", target_dist=1.0, wall_gap=self.config.WALL_GAP_M)
        self.wall_hug_advance(
            side="right",
            target_dist=1.1,
            wall_gap=self.config.WALL_GAP_M,
            stop_at_wall=True,
            switch_dist=1.1,
        )

        print(">>> [chain] final turn")
        target_yaw = (self.odom.y + 90.0) % 360.0
        self.turn_to(target_yaw, tolerance=3.0, yaw_speed=0.55)
        self.init_angle = target_yaw
        self.publish_gait(0, height=0.08, pos_z=0.07)
        time.sleep(0.5)

        self.dash_centered(1.0)
        self.act(0)
        time.sleep(1.0)
        self.relax()

        self.task_id = TaskId.DONE
        self.stage = RaceStage.FINISHED


def main():
    rclpy.init()
    master = RaceNewMaster(init_angle=0)

    executor = MultiThreadedExecutor()
    executor.add_node(master.lidar)
    Thread(target=executor.spin, daemon=True).start()

    try:
        master.run_competition()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
