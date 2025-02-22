import rclpy
from rclpy.node import Node
from protocol.msg import HeadTofPayload
from threading import Thread
import time

class HeadTofSubscriber(Node):

    def __init__(self):
        super().__init__('head_tof_subscriber')
        self.subscription = self.create_subscription(
            HeadTofPayload,
            '/mi_desktop_48_b0_2d_7b_02_dc/head_tof_payload',
            self.listener_callback,
            10)
        self.left_data_matrix = None
        self.right_data_matrix = None

    def listener_callback(self, msg):
        self.left_data_matrix = self.format_to_8x8(msg.left_head.data)
        self.right_data_matrix = self.format_to_8x8(msg.right_head.data)

    def format_to_8x8(self, data):
        return [data[i:i+8] for i in range(0, len(data), 8)]

    def tof_gaotai(self):
        count = 0
        for col in range(8):
            num_values = sum(1 for row in self.left_data_matrix if round(row[col], 3) in (0.050, 0.560) or row[col]>0.40)
            if num_values >= 4:
                count += 1
        for col in range(8):
            num_values = sum(1 for row in self.right_data_matrix if round(row[col], 3) in (0.050, 0.560) or row[col]>0.40)
            if num_values >= 4:
                count += 1
        print("count:"+str(count))
        return count < 14

    def tof_dumuqiao(self):
        count_left = 0
        count_right=0
        for row in self.left_data_matrix:
            num_values = sum(1 for value in row if round(value, 3) in (0.560, 0.050))
            if num_values >= 4:
                count_left += 1

        for row in self.right_data_matrix:
            num_values = sum(1 for value in row if round(value, 3) in (0.560, 0.050))
            if num_values >= 7:
                count_right += 1

        if count_left>1:
            return 1
        elif count_right>1:
            return -1
        else:
            return 0

def get_head_tof_node():
    head_tof_node = HeadTofSubscriber()

    def spin_node():
        try:
            rclpy.spin(head_tof_node)
        except KeyboardInterrupt:
            pass
        finally:
            head_tof_node.destroy_node()

    spin_thread = Thread(target=spin_node)
    spin_thread.start()
    return head_tof_node





# 使用示例
if __name__ == '__main__':
    rclpy.init(args=None)
    head_tof_node = get_head_tof_node()
    while True:
        while head_tof_node.left_data_matrix:
            print(f"gaotai:{head_tof_node.tof_dumuqiao()}")
            time.sleep(1)
