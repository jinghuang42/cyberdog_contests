import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
import os
import sys
import subprocess

# --- 【黑科技：自动寻找并注入 protocol 路径】 ---
def try_import_protocol():
    try:
        # 尝试直接导入
        from protocol.msg import HeadTofPayload
        return HeadTofPayload, True
    except ImportError:
        try:
            # 报错时，尝试通过系统命令寻找包路径
            prefix = subprocess.check_output(['ros2', 'pkg', 'prefix', 'protocol']).decode().strip()
            # 根据 ROS2 惯例，Python 包通常在下述路径
            path = os.path.join(prefix, 'lib/python3.8/site-packages')
            if path not in sys.path:
                sys.path.append(path)
            from protocol.msg import HeadTofPayload
            return HeadTofPayload, True
        except Exception as e:
            print(f">>> [严重警告] 无法通过路径注入导入 TOF 协议: {e}")
            return None, False

# 执行导入
HeadTofPayload, TOF_AVAILABLE = try_import_protocol()

class ROSListener(Node):
    def __init__(self):
        super().__init__('ros_listener')
        self.scan = None
        self.tof_l = 1.0
        self.tof_r = 1.0

        # 1. 雷达订阅
        scan_topic = '/mi_desktop_48_b0_2d_5f_b8_ce/scan'
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(LaserScan, scan_topic, self.listener_callback, qos)

        # 2. TOF 订阅 (仅当协议可用时)
        if TOF_AVAILABLE:
            tof_topic = '/mi_desktop_48_b0_2d_5f_b8_ce/head_tof_payload'
            self.tof_subscription = self.create_subscription(
                HeadTofPayload,
                tof_topic,
                self.tof_callback,
                10)
            self.get_logger().info('>>> [系统] 雷达与 TOF 订阅成功启动')
        else:
            self.get_logger().error('>>> [系统] TOF 协议不可用，居中功能将失效')

    def listener_callback(self, msg):
        self.scan = msg.ranges

    def tof_callback(self, msg):
        """解析 TOF 左右深度值"""
        try:
            # 兼容性处理：msg 可能叫 left_head 或 head
            head_data = None
            if hasattr(msg, 'left_head'):
                head_data = msg.left_head.data
            elif hasattr(msg, 'head'):
                head_data = msg.head.data
            
            if head_data and len(head_data) >= 8:
                # 取前两个均值和后两个均值
                self.tof_l = (head_data[0] + head_data[1]) / 2.0
                self.tof_r = (head_data[-1] + head_data[-2]) / 2.0
        except Exception:
            pass

    def get_tof(self):
        return self.tof_l, self.tof_r

    def get_scan(self):
        if self.scan is None: return None
        def get_mean(data_slice):
            valid = [d for d in data_slice if d > 0.05 and d < 10.0]
            return sum(valid) / len(valid) if valid else float('inf')
        try:
            right_mean = get_mean(self.scan[0:10])
            front_mean = get_mean(self.scan[len(self.scan)//2-5 : len(self.scan)//2+5])
            left_mean = get_mean(self.scan[-11:-1])
            return [left_mean, front_mean, right_mean, 0.0, 0.0]
        except: return None
