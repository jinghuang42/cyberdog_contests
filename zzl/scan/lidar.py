import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserScanSubscriber(Node):

    def __init__(self):
        super().__init__('laser_scan_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/mi_desktop_48_b0_2d_5f_b8_ce/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print(msg.ranges[0:9])
        print(msg.ranges[len(msg.ranges) // 2-5:len(msg.ranges) // 2+5])
        print(msg.ranges[-10:-1])
        # 计算左侧、正前方和右侧的距离
        right_distance = [d for d in msg.ranges[0:9] if d!=0]
        right_mean=sum(right_distance)/len(right_distance)
        front_distance =[d for d in msg.ranges[len(msg.ranges) // 2-5:len(msg.ranges) // 2+5] if d!=0]  
        front_mean=sum(front_distance)/len(front_distance)
        left_distance = [d for d in msg.ranges[-10:-1] if d!=0] 
        left_mean=sum(left_distance)/len(left_distance)
        # 输出距离信息
        self.get_logger().info(f'Left Distance: {left_mean}')
        self.get_logger().info(f'Front Distance: {front_mean}')
        self.get_logger().info(f'Right Distance: {right_mean}')

def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin(laser_scan_subscriber)
    laser_scan_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()