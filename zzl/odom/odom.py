import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/mi_desktop_48_b0_2d_7b_02_dc/odom_out',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用的变量警告

    def listener_callback(self, msg):
        # 打印位置信息
        self.get_logger().info(f'Position - x: {msg.pose.pose.position.x}, y: {msg.pose.pose.position.y}, z: {msg.pose.pose.position.z}')
        
        # 打印方向信息
        self.get_logger().info(f'Orientation - x: {msg.pose.pose.orientation.x}, y: {msg.pose.pose.orientation.y}, z: {msg.pose.pose.orientation.z}, w: {msg.pose.pose.orientation.w}')
        
        # 打印线速度和角速度
        self.get_logger().info(f'Linear Velocity - x: {msg.twist.twist.linear.x}, y: {msg.twist.twist.linear.y}, z: {msg.twist.twist.linear.z}')
        self.get_logger().info(f'Angular Velocity - x: {msg.twist.twist.angular.x}, y: {msg.twist.twist.angular.y}, z: {msg.twist.twist.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()
    rclpy.spin(odom_subscriber)
    odom_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()