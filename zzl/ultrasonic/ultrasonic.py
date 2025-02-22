import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class UltrasonicSubscriber(Node):

    def __init__(self):
        super().__init__('ultrasonic_subscriber')
        # 创建订阅者
        self.subscription = self.create_subscription(
            Range,
            '/mi_desktop_48_b0_2d_7b_02_dc/ultrasonic_payload',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量警告

    def listener_callback(self, msg):
        # 输出超声波传感器数据
        self.get_logger().info(f"Range: {msg.range} m")

def main(args=None):
    # 初始化rclpy
    rclpy.init(args=args)
    # 创建节点
    ultrasonic_subscriber = UltrasonicSubscriber()
    # 运行节点
    rclpy.spin(ultrasonic_subscriber)
    # 销毁节点
    ultrasonic_subscriber.destroy_node()
    # 关闭rclpy
    rclpy.shutdown()

if __name__ == '__main__':
    main()