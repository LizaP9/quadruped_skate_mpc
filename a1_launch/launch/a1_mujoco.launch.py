import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    package_dir = get_package_share_directory('a1_launch')
    viewer_script = os.path.join(package_dir, 'scripts', 'a1_mujoco_viewer.py')

    mujoco_viewer_process = ExecuteProcess(
        cmd=["python3", viewer_script],
        output='screen'
    )

    foot_trajectory_node = Node(
        package='a1_trajectory_planning',
        executable='foot_trajectory_planner_node',
        name='foot_trajectory_planner',
        output='screen'
    )

    return LaunchDescription([
        mujoco_viewer_process,
        foot_trajectory_node
    ])