from datetime import datetime
import subprocess

def run_with_timestamp(command, output_file):
    # 使用 datetime.now() 获取精确到毫秒的时间
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 去掉最后3位以保留毫秒
    with open(output_file, 'a') as f:
        f.write(f"\n[{timestamp}] Running: {command}\n")
        process = subprocess.Popen(command, shell=True, stdout=f, stderr=f)
        process.wait()

if __name__ == "__main__":
    # 定义两个子文件的launch命令
    sub_file_1_command = "ros2 launch my_mid360 mid360_bringup.launch.py"
    sub_file_2_command = "ros2 launch my_realsense realsense_bringup.launch.py"

    # 定义输出日志文件
    output_file_1 = "sub_file_1_output.log"
    output_file_2 = "sub_file_2_output.log"

    # 启动两个子文件并记录时间戳
    run_with_timestamp(sub_file_1_command, output_file_1)
    run_with_timestamp(sub_file_2_command, output_file_2)
