import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/image_rgb',
            self.listener_callback,
            10)
        self.received_image = False
        self.image = None

    def listener_callback(self, msg):
        if not self.received_image:
            self.image = np.array(msg.data, dtype=np.uint8).reshape((480, 640, 3))
            self.get_logger().info('Image captured.')
            self.received_image = True

def save_image(image,filename):
    flattened_image = image.flatten()
    np.savetxt(filename, flattened_image, fmt='%d')

def image_data_func():
    image_subscriber = ImageSubscriber()

    while rclpy.ok() and not image_subscriber.received_image:
        rclpy.spin_once(image_subscriber)  # 使用spin_once以允许手动控制循环

    image_subscriber.destroy_node()  # 手动销毁节点
    
    return image_subscriber.image

if __name__ == '__main__':
    image_data_func()