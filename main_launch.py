from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 激光雷达的 launch 文件路径
    mid360_launch_path = get_package_share_directory('my_mid360') + '/launch/mid360_bringup.launch.py'

    # 深度相机的 launch 文件路径
    realsense_launch_path = get_package_share_directory('my_realsense') + '/launch/realsense_bringup.launch.py'

    # 包含激光雷达 launch
    mid360_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mid360_launch_path)
    )

    # 包含深度相机 launch
    realsense_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path)
    )

    # 返回联合启动描述
    return LaunchDescription([
        mid360_bringup,
        realsense_bringup,
    ])
