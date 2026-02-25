import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, DeclareLaunchArgument, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_auto_simulation = get_package_share_directory('auto_simulation')
    pkg_auto_robot = get_package_share_directory('auto_robot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # World path
    world_path = os.path.join(pkg_auto_simulation, 'worlds', 'track_01.world')

    # Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )
    
    # ... (Robot State Publisher - Unchanged)
    
    # We need to define xacro_file and controller_config again because they were deleted
    xacro_file = os.path.join(pkg_auto_robot, 'urdf', 'robot.urdf.xacro')
    controller_config = os.path.join(pkg_auto_robot, 'config', 'robot_controller.yaml')
    
    # from launch.substitutions import Command # Moved to top
    robot_description_content = Command([
        'xacro ', xacro_file, 
        ' controller_config:=', controller_config
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}],
    )

    # Spawn Entity
    # Position: x=0.0, y=0.0, z=0.2 (Start of road)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'auto_robot',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.2',
                   '-Y', '0.0'], # Facing East (+X)
        output='screen'
    )

    # Controllers
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_group_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_velocity_controller", "--controller-manager", "/controller_manager"],
    )
    
    use_driver = LaunchConfiguration('use_driver')
    declare_use_driver = DeclareLaunchArgument(
        'use_driver',
        default_value='true',
        description='Whether to launch the omnidirectional driver'
    )

    use_teleop = LaunchConfiguration('use_teleop')
    declare_use_teleop = DeclareLaunchArgument(
        'use_teleop',
        default_value='false',
        description='Whether to launch teleop_twist_keyboard in a new terminal'
    )

    # Omni driver parameters - explicit dict to avoid namespace issues with YAML file
    omni_params = {
        'wheel_names': ['joint_1', 'joint_2', 'joint_3'],
        'robot_radius': 0.124,
        'wheel_radius': 0.0325,
        'wheel_angles_deg': [270.0, 30.0, 150.0],
        'roller_angle_deg': 0.0,
        'use_field_centric': False,
        'use_sim_time': True
    }
    
    omni_driver = Node(
        package='omnidirectional_driver',
        executable='omni_driver',
        name='omnidirectional_driver',
        output='screen',
        parameters=[omni_params]
    )

    # Delays to prevent race conditions (Same as auto_robot launch)
    load_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=5.0,
                    actions=[joint_state_broadcaster]
                )
            ],
        )
    )

    load_jgvc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[joint_group_velocity_controller],
        )
    )

    load_driver_after_jgvc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_group_velocity_controller,
            on_exit=[omni_driver],
        )
    )
    
    # Teleop Node (in new terminal)
    # from launch.actions import ExecuteProcess # Moved to top
    
    teleop_cmd = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
        condition=IfCondition(use_teleop)
    )

    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz'
    )

    rviz_config_file = os.path.join(pkg_auto_simulation, 'rviz', 'view_robot.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz)
    )
    # 1. Define the Fusion Node
    combined_odom_node = Node(
        package='auto_simulation', # or 'auto_robot' depending on where you put it
        executable='combined_odom.py',
        name='manual_fusion_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        declare_use_driver,
        declare_use_teleop,
        declare_use_rviz,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_jsb_after_spawn,
        load_jgvc_after_jsb,
        load_driver_after_jgvc,
        teleop_cmd,
        rviz_node,
        combined_odom_node
    ])
