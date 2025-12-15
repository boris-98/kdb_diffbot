import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
# bellow two imports are used for the delayed node spawning
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command



def generate_launch_description():

    package_name='kdb_diffbot'

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')
    map_file_path = os.path.join(get_package_share_directory(package_name),'maps','test_map.png')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file],
    )

    delayed_controller_manager = TimerAction(
        period=5.0,  # seconds
        actions=[controller_manager],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    # This is used to delay the spawning of the diff drive controller until the robot is fully loaded in Gazebo
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # This is used to delay the spawning of the joint broad controller until the robot is fully loaded in Gazebo
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )


    # Launch them all!
    return LaunchDescription([
        rsp,
        # joystick,
        # twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner, # bili su obicni samo bez delayed
        delayed_joint_broad_spawner
    ])
