import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time
from threading import Thread
class ROSListener(Node):
    def __init__(self):
        super().__init__('ros_listener')
        self.scan = None
        
        # 创建 ROS 2 订阅者
        self.subscription = self.create_subscription(
            LaserScan,
            '/mi_desktop_48_b0_2d_7b_02_dc/scan',
            self.listener_callback,
            10)
        self.subscription
    def listener_callback(self, msg):
        self.scan=msg.ranges
        
    def get_scan(self):
        right_distance = [d for d in self.scan[0:10] if d > 0.05]

        right_mean = sum(right_distance) / len(right_distance) if right_distance else float('inf')

        right_front_distance=[d for d in self.scan[54:64] if d >0.05]
        right_front_mean=sum(right_front_distance) / len(right_front_distance) if right_front_distance else float('inf')

        front_distance = [d for d in self.scan[len(self.scan) // 2 - 5:len(self.scan) // 2 + 5] if d > 0.05]
        front_mean = sum(front_distance) / len(front_distance) if front_distance else float('inf')
        
        left_front_distance=[d for d in self.scan[-64:-54] if d >0.05]
        left_front_mean=sum(left_front_distance) / len(left_front_distance) if left_front_distance else float('inf')

        left_distance = [d for d in self.scan[-11:-1] if d > 0.05]
        left_mean = sum(left_distance) / len(left_distance) if left_distance else float('inf')
        return [left_mean, front_mean, right_mean,left_front_mean,right_front_mean]

def start_scan_listener():
    ros_listener = ROSListener()
    def spin_node():
        try:
            rclpy.spin(ros_listener)
        except KeyboardInterrupt:
            pass
        finally:
            ros_listener.destroy_node()
    spin_thread = Thread(target=spin_node)
    spin_thread.start()
    return ros_listener
if __name__ == '__main__':
    rclpy.init(args=None)
    ros_listener = start_scan_listener()
    while True:
        time.sleep(1)
    