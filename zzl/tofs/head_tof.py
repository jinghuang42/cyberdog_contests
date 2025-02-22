import rclpy
from rclpy.node import Node
from protocol.msg import HeadTofPayload  # 确保导入正确的消息类型

class HeadTofSubscriber(Node):

    def __init__(self):
        super().__init__('head_tof_subscriber')
        # 创建订阅者
        self.subscription = self.create_subscription(
            HeadTofPayload,
            '/mi_desktop_48_b0_2d_7b_02_dc/head_tof_payload',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量警告

    def listener_callback(self, msg):
        # 处理左前TOF数据
        if msg.left_head.data_available:
            left_data_matrix = self.format_to_8x8(msg.left_head.data)
            self.get_logger().info("Left Head TOF Data (8x8):")
            for row in left_data_matrix:
                self.get_logger().info(f"{' '.join(f'{value:.3f}' for value in row)}")
            self.get_logger().info("-" * 40)  # 分隔符

        # 处理右前TOF数据
        if msg.right_head.data_available:
            right_data_matrix = self.format_to_8x8(msg.right_head.data)
            self.get_logger().info("Right Head TOF Data (8x8):")
            for row in right_data_matrix:
                self.get_logger().info(f"{' '.join(f'{value:.3f}' for value in row)}")
            self.get_logger().info("-" * 40)  # 分隔符

    def format_to_8x8(self, data):
        """将一维数据转换为8x8矩阵"""
        return [data[i:i+8] for i in range(0, len(data), 8)]

    
def main(args=None):
    # 初始化rclpy
    rclpy.init(args=args)
    # 创建节点
    head_tof_subscriber = HeadTofSubscriber()
    # 运行节点
    rclpy.spin(head_tof_subscriber)
    # 销毁节点
    head_tof_subscriber.destroy_node()
    # 关闭rclpy
    rclpy.shutdown()

if __name__ == '__main__':
    main()