import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class UltrasonicSubscriber(Node):

    def __init__(self):
        super().__init__('ultrasonic_subscriber')
        self.subscription = self.create_subscription(
            Range,
            '/mi_desktop_48_b0_2d_7b_02_dc/ultrasonic_payload',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # 输出测量到的距离
        self.get_logger().info(f'Received ultrasonic range: {msg.range:.2f} meters')

def main(args=None):
    rclpy.init(args=args)
    ultrasonic_subscriber = UltrasonicSubscriber()
    rclpy.spin(ultrasonic_subscriber)
    ultrasonic_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()