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

def image_data_func(image): #bgr
    left_half = image[:, :640 // 2]
    right_half = image[:, 640 // 2:]
    left_r_mean = np.mean(left_half[:, :, 2])
    # left_g_mean = np.mean(left_half[:, :, 1])
    right_r_mean = np.mean(right_half[:, :, 2])
    # right_g_mean = np.mean(right_half[:, :, 1])
    return  left_r_mean<right_r_mean

if __name__ == '__main__':
    image_data_func()