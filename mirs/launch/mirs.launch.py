import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, LogInfo, ExecuteProcess
from launch.substitutions import FindExecutable
from launch.event_handlers import OnProcessStart
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    esp_port = DeclareLaunchArgument(
        'esp_port', default_value='/dev/ttyUSB1',
        description='Set esp32 usb port.')
    lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0',
        description='Set lidar usb port.')
    
    # YAMLファイルのパス（エラーが起きるときは絶対パスに変更を推奨）
    config_file_path = os.path.join(get_package_share_directory('mirs'),'config','config.yaml')
    #config_file_path = your_file_path

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
        arguments=['serial', '--dev', LaunchConfiguration('esp_port'), '-v6']
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_s1_launch.py')
        ),
        launch_arguments={'serial_port': LaunchConfiguration('lidar_port')}.items()
    )

    tf2_ros_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=["0", "0", "0.35", "-1.57", "0", "0", "base_link", "laser"]
    )

    pre_update = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' run',
            ' mirs',
            ' update',
            ' --ros-args',
            ' --params-file ',
            config_file_path
        ]],
        shell=True
    )

    parameter_change_action = RegisterEventHandler(
        OnProcessStart(
            target_action=micro_ros,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[
                        pre_update,
                        LogInfo(msg='Change the parameter of the node')
                    ],
                )
            ]
        )
    )

    ld = LaunchDescription()
    ld.add_action(esp_port)
    ld.add_action(lidar_port)

    ld.add_action(odometry_node)
    ld.add_action(parameter_node)
    ld.add_action(micro_ros)
    ld.add_action(sllidar_launch)
    ld.add_action(tf2_ros_node)

    ld.add_action(parameter_change_action)

    return ld