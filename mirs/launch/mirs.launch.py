from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # YAMLファイルのパスを適切に展開
    config_file_path = os.path.expanduser('~/mirs2403/src/mirs_mg5/mirs/config/config.yaml')

    odometry_node = Node(
        package='mirs',
        executable='odometry_publisher',
        name='odometry_publisher',
        output='screen',
        parameters=[config_file_path]  # 修正点: カンマを削除して適切にリストとして指定
    )

    parameter_node = Node(
        package='mirs',
        executable='parameter_publisher',
        name='parameter_publisher',
        output='screen',
        parameters=[config_file_path]  # 修正点: カンマを削除して適切にリストとして指定
    )


    micro_ros = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyUSB1', '-v6']
    )
    
    # odometry_nodeとmicro_rosの両方をLaunchDescriptionに追加
    return LaunchDescription([
        odometry_node,
	parameter_node
        parameter_node,

        micro_ros  # ここでmicro_rosを追加
    ])
