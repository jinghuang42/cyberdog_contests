import os
import signal
import subprocess

def kill_processes_by_name(process_name):
    try:
        # 使用ps命令查找所有包含指定名称的进程
        result = subprocess.run(['ps', 'aux'], stdout=subprocess.PIPE)
        processes = result.stdout.decode('utf-8').splitlines()

        # 遍历所有进程，找到包含process_name的进程
        for process in processes:
            if process_name in process and 'grep' not in process:
                # 获取PID并终止进程
                pid = int(process.split()[1])
                print(f"Killing process {process_name} with PID: {pid}")
                os.kill(pid, signal.SIGTERM)  # 温和地终止进程
                os.kill(pid, signal.SIGKILL)  # 强制终止（如果需要）
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    # 关闭所有stereo_camera相关的进程
    kill_processes_by_name('stereo_camera')
    kill_processes_by_name('realsense2_camera')