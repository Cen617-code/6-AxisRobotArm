import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_arm_demo')
    
    # Process URDF file
    xacro_file = os.path.join(pkg_share, 'urdf', '6_axis_arm.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}
    
    # Robot State Publisher (must use sim time to match Gazebo clock)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Launch Gazebo physics server (headless)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -s -v4 empty.sdf'}.items(),
    )

    # Launch Gazebo GUI client separately
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '-v4'],
        output='screen'
    )

    # Spawn entity via ros_gz_sim create
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', '6_axis_arm', '-z', '0.01'],
        output='screen'
    )

    # Bridge clock from Gazebo to ROS
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Load and start joint state broadcaster
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Load and start arm controller
    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    # Wait for spawn_entity to finish before loading controllers
    delay_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster, load_arm_controller],
        )
    )

    # Delay GUI launch by 5 seconds so physics + controllers are ready first
    delayed_gui = TimerAction(
        period=5.0,
        actions=[gazebo_gui]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo_server,
        spawn_entity,
        clock_bridge,
        delay_broadcaster_after_spawn,
        delayed_gui,
    ])
