import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('auto_robot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='')
    
    # Process xacro to get robot description
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Controller config
    controller_config = os.path.join(pkg_share, 'config', 'robot_controller.yaml')

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Launch Gazebo (without extra args - plugin will get params from controller_manager)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': 'true'
        }.items()
    )

    # Spawn robot entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'auto_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )



    # Spawn joint state broadcaster controller (after spawn)
    spawn_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Spawn velocity controller
    spawn_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_group_velocity_controller'],
        output='screen'
    )

    # Event handlers to sequence controller loading (wait 3 seconds after spawn)
    # This prevents "Error loading controller" if called too early
    load_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[spawn_joint_state_broadcaster]
                )
            ],
        )
    )

    load_velocity_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_velocity_controller],
        )
    )

    # Omnidirectional Driver Node (after controllers are loaded)
    omni_driver_node = Node(
        package='omnidirectional_driver',
        executable='omni_driver',
        name='omnidirectional_driver',
        output='screen',
        parameters=[
            controller_config,
            {'use_sim_time': True}
        ]
    )

    load_driver_after_velocity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_velocity_controller,
            on_exit=[omni_driver_node],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='World file to load in Gazebo'
        ),
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        load_jsb_after_spawn,
        load_velocity_after_jsb,
        load_driver_after_velocity,
    ])
