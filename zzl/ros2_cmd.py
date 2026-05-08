import rclpy
from rclpy.node import Node
from protocol.msg import MotionServoCmd  # 请确保这是实际的消息类型

class MotionStatusPublisher(Node):

    def __init__(self):
        super().__init__('motion_status_publisher')
        self.publisher_ = self.create_publisher(MotionServoCmd, '/mi_desktop_48_b0_2d_5f_b8_ce/motion_servo_cmd', 10)
        timer_period = 1.0  # 每2秒发布一次
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = MotionServoCmd()
        msg.motion_id =101  # 示例数据，根据需要修改
        msg.cmd_type=1
        msg.cmd_source=0
        msg.value=2
        msg.vel_des=[1.0,0.0,0.0]
        msg.rpy_des=[0.0,0.0,0.0]
        msg.pos_des=[0.0,0.0,0.0]
        msg.acc_des=[0.0,0.0,0.0]
        msg.ctrl_point=[0.0,0.0,0.0]
        msg.foot_pose=[0.0,0.0,0.0]
        msg.step_height=[0.10,0.10,]
        # 设置其他必要的字段
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: motion_id={msg.motion_id}')

def main(args=None):
    rclpy.init(args=args)
    motion_status_publisher = MotionStatusPublisher()
    rclpy.spin(motion_status_publisher)
    motion_status_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()