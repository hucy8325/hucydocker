import os
import time
import subprocess

def run_with_timestamp(command, output_file):
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
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

def compare_outputs(file1, file2):
    with open(file1, 'r') as f1, open(file2, 'r') as f2:
        lines1 = f1.readlines()
        lines2 = f2.readlines()

    # 对比逐行输出
    for i, (line1, line2) in enumerate(zip(lines1, lines2), 1):
        if line1.strip() != line2.strip():
            print(f"Difference at line {i}:\nFile1: {line1.strip()}\nFile2: {line2.strip()}")

if __name__ == "__main__":
    compare_outputs("sub_file_1_output.log", "sub_file_2_output.log")