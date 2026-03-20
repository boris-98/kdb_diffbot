import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
# bellow two imports are used for the delayed node spawning
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='kdb_diffbot' #<--- CHANGE ME
    map_file_path = os.path.join(get_package_share_directory(package_name),'maps','test_map.png')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # joystick = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','joystick.launch.py'
    #             )]), launch_arguments={'use_sim_time': 'true'}.items()
    # )

    # twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    # twist_mux = Node(
    #         package="twist_mux",
    #         executable="twist_mux",
    #         parameters=[twist_mux_params, {'use_sim_time': True}],
    #         remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    #     )

    # gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'kdb_warehouse_more_tags.sdf'
        )    
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-z', '0.1'],
                        output='screen')

    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    # This is used to delay the spawning of the diff drive controller until the robot is fully loaded in Gazebo
    delayed_diff_drive_spawner = TimerAction(
        period=10.0,
        actions=[diff_drive_spawner]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # This is used to delay the spawning of the joint broad controller until the robot is fully loaded in Gazebo
    delayed_joint_broad_spawner = TimerAction(
        period=10.0,
        actions=[joint_broad_spawner]
    )

    # Get the path to the package share directory
    apriltag_ros_share_dir = get_package_share_directory('apriltag_ros')

    # Get the path to the tags_36h11.yaml file
    tags_36h11_yaml_file = os.path.join(apriltag_ros_share_dir, 'cfg', 'tags_36h11.yaml')
    print("AprilTag parameter file path:", tags_36h11_yaml_file)

    apriltag_ros_spawner = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info')
        ],
        parameters=[tags_36h11_yaml_file]
    )
    delayed_apriltag_ros_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[apriltag_ros_spawner],
        )
    )

    ekf_node = Node(
        package='kdb_diffbot',
        executable='ekf_localization_node.py',  # or 'ekf_localization' if not .py
        name='ekf_localization',
        output='screen',
        parameters=[{
            'tag_map_yaml': os.path.join(get_package_share_directory(package_name), 'config', 'warehouse_tag_map.yaml'),#'tag_map.yaml'),
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'use_sim_time': True
        }]
    )
    
    g2o_generator_node = Node(
        package='kdb_diffbot',
        executable='g2o_generator_node.py',  
        name='g2o_generator',
        output='screen',
        parameters=[{
            'tag_map_yaml': os.path.join(
                get_package_share_directory(package_name),
                'config',
                'maze_tag_map.yaml'
            ),
            'use_sim_time': True
        }]
    )

    delayed_ekf_node_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[ekf_node],
        )
    )
    
    delayed_g2o_generator_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[g2o_generator_node],
        )
    )
    
    g2o_poseonly_generator_node = Node(
        package='kdb_diffbot',
        executable='g2o_poseonly_generator_node.py',  
        name='g2o_poseonly_generator',
        output='screen',
        parameters=[{
            'tag_map_yaml': os.path.join(
                get_package_share_directory(package_name),
                'config',
                'maze_tag_map.yaml'
            ),
            'use_sim_time': True
        }]
    )
    
    delayed_g2o_poseonly_generator_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[g2o_poseonly_generator_node],
        )
    )

    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner



    # Launch them all!
    return LaunchDescription([
        SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1'),        # ako treba na GPU
        SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia'),   # ako treba na GPU
        SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='false'),   # ako treba na GPU
        rsp,
        # joystick,
        # twist_mux,
        world_arg,
        gazebo,
        spawn_entity,
        delayed_diff_drive_spawner, # bili su obicni samo bez delayed
        delayed_joint_broad_spawner,
        delayed_apriltag_ros_spawner,
        # delayed_ekf_node_spawner,
        delayed_g2o_generator_node,
        # delayed_g2o_poseonly_generator_node,
        ros_gz_bridge,
        ros_gz_image_bridge 
    ])
