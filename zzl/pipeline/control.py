import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import subprocess
import time
import numpy as np
from rgb import image_data_func,save_image
from kill import  kill_processes_by_name
def start_camera():
    subprocess.Popen(['ros2', 'launch', 'realsense2_camera', 'on_dog.py'],stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)  #非阻塞
    time.sleep(3)  #等待节点启动
    subprocess.run(['ros2', 'lifecycle', 'set', '/camera/camera', 'configure'])
    subprocess.run(['ros2', 'lifecycle', 'set', '/camera/camera', 'activate'])

    subprocess.Popen(['ros2', 'run', 'camera_test', 'stereo_camera'])  # 非阻塞
    time.sleep(3)  # 等待节点启动
    
    subprocess.run(['ros2', 'lifecycle', 'set', '/stereo_camera', 'configure'])

def deactivate_camera():
    subprocess.run(['ros2', 'lifecycle', 'set', '/stereo_camera', 'deactivate'])
def activate_camera():
    subprocess.run(['ros2', 'lifecycle', 'set', '/stereo_camera', 'activate'])


def stop_camera():
    subprocess.run(['ros2', 'lifecycle', 'set', '/camera/camera', 'deactivate'])
    kill_processes_by_name("stereo_camera")
    kill_processes_by_name("realsense2_camera")


def get_image(queue):
    try:
        rclpy.init()
        activate_camera()
        res = image_data_func()
        save_image(res,"zzl.txt")
        deactivate_camera()
        rclpy.shutdown()
        return queue.put(res)
    except Exception as e:
        print(e)
    finally:
        rclpy.shutdown()


def get_image2(file_name):
    rclpy.init()
    activate_camera()
    res = image_data_func()
    save_image(res,file_name)
    print(res)
    deactivate_camera()
    rclpy.shutdown()
    return res

# 这是主函数，你可以根据需要调用它来启动和关闭相机服务
def main():

    start_camera()

    #第一张
    get_image2("1.txt")

    #第二张
    get_image2("2.txt")


    #第三张
    get_image2("3.txt")


    # 关闭相机服务
    stop_camera()    


if __name__ == '__main__':
    main()