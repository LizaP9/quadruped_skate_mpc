import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
# from launch.actions import ExecuteProcess, RegisterEventHandler
# from launch.event_handlers import OnProcessExit, OnProcessStart

from launch_ros.actions import Node

import xacro

def generate_launch_description():
    mujoco_ros2_control_demos_path = os.path.join(
        get_package_share_directory('mujoco_ros2_control_demos'))
    
    xacro_file = os.path.join(mujoco_ros2_control_demos_path,
                              'a1_description',
                              'a1.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}


    controller_config_file = os.path.join(mujoco_ros2_control_demos_path, 'config', 'a1.yaml')

    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config_file,
            {'mujoco_model_path':os.path.join(mujoco_ros2_control_demos_path, 'a1_description', 'scene.xml')}
        ]
    )

    node_robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[robot_description],
    output='screen'
    )



    return LaunchDescription([
        node_mujoco_ros2_control,
        node_robot_state_publisher
    ])
