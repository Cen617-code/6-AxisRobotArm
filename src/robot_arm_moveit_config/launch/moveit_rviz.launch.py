import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def load_yaml(package_name, file_path):
    full_path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_config_pkg = 'robot_arm_moveit_config'
    robot_arm_pkg = 'robot_arm_demo'

    # --- Robot Description ---
    urdf_file = os.path.join(
        get_package_share_directory(robot_arm_pkg), 'urdf', '6_axis_arm.urdf.xacro')
    doc = xacro.process_file(urdf_file)
    robot_description = doc.toxml()

    robot_description_param = {'robot_description': robot_description}

    # --- SRDF ---
    srdf_file = os.path.join(
        get_package_share_directory(moveit_config_pkg), 'config', '6_axis_arm.srdf')
    with open(srdf_file, 'r') as f:
        srdf_content = f.read()

    robot_description_semantic = {'robot_description_semantic': srdf_content}

    # --- Kinematics ---
    kinematics_yaml = load_yaml(moveit_config_pkg, 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # --- Joint Limits ---
    joint_limits_yaml = load_yaml(moveit_config_pkg, 'config/joint_limits.yaml')
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}

    # --- OMPL ---
    ompl_yaml = load_yaml(moveit_config_pkg, 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'ompl': ompl_yaml,
    }

    # --- MoveIt Controllers ---
    moveit_controllers_yaml = load_yaml(moveit_config_pkg, 'config/moveit_controllers.yaml')
    moveit_controllers = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    moveit_controllers.update(moveit_controllers_yaml)

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 2.0,
        'trajectory_execution.allowed_goal_duration_margin': 1.0,
        'trajectory_execution.allowed_start_tolerance': 0.0,
    }

    common_params = [
        robot_description_param,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_planning_pipeline_config,
        moveit_controllers,
        trajectory_execution,
        {'use_sim_time': True},
    ]

    # --- Move Group Node ---
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=common_params,
    )

    # --- RViz2 ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        parameters=common_params,
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])
