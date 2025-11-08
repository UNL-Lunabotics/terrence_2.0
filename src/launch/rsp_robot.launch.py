import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitution import Command

from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'terrence_2'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    
    world = LaunchConfiguration('world')
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='src/worlds/obstacles.sdf',
        description='World to load'
    )
    
    robot_description = Command(['ros2 params get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description' : robot_description}, controller_params_file]
    )
    
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ]
    )
    
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel')]
    )
    
    image_compressor = Node(
        package='image_transport',
        executable='republish',
        arguments=[
            'raw', 'compressed',
            '--ros-args', '-r', 'in:=/camera/image_raw', '-r', 'out:=/camera/camera/image_raw'
        ]
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        delayed_controller_manager,
        diff_drive_spawner,
        joint_broad_spawner,
        # teleop,
        # world_arg,
        # ros_gz_bridge,
        # image_compressor
    ])