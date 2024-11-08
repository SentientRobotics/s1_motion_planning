from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
from launch.actions import TimerAction

def load_yaml(package_name, *path):
    package_path = os.path.join(get_package_share_directory(package_name), *path)
    with open(package_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("s1_moveit_config"), "config")
    
    urdf_file = os.path.join(config_dir, "sentients1.urdf.xacro")
    srdf_file = os.path.join(config_dir, "sentients1.srdf")
    
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")
    
    if not os.path.exists(srdf_file):
        raise FileNotFoundError(f"SRDF file not found: {srdf_file}")
    
    moveit_config = (
        MoveItConfigsBuilder("sentients1", "robot_description", "s1_moveit_config")
        .robot_description(file_path=urdf_file)
        .robot_description_semantic(file_path=srdf_file)
        .trajectory_execution(file_path=os.path.join(config_dir, "moveit_controllers.yaml"))
        .to_moveit_configs()
    )

    ompl_planning_pipeline_config = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl','pilz_industrial_motion_planner'],
        'ompl': {},
        'chomp': {},
        'pilz_industrial_motion_planner': {},
    }

    moveit_config_package_name = 's1_moveit_config'
    ompl_planning_pipeline_config['ompl'].update(load_yaml(moveit_config_package_name, 'config', 'ompl_planning.yaml'))
    ompl_planning_pipeline_config['pilz_industrial_motion_planner'].update(load_yaml(moveit_config_package_name, 'config', 'pilz_industrial_motion_planner_planning.yaml'))
    
    # Adjusted launch order to prioritize core components
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[os.path.join(config_dir, "ros2_controllers.yaml")],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
            )
        ]
    )

    # Adding delay to ensure controllers are loaded in order
    arm_controller = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller", "-c", "/controller_manager"],
                output="screen",
            )
        ]
    )

    hand_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["hand_controller", "-c", "/controller_manager"],
                output="screen",
            )
        ]
    )

    move_group_node = TimerAction(
        period=8.0,  # Adjusted to launch before RViz
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {"move_group/arm": {
                        "joints": ["Linear_Joint", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
                    }},
                    {"move_group/hand": {
                        "joints": ["right_index_1_joint", "right_little_1_joint", "right_middle_1_joint", "right_ring_1_joint", "right_thumb_1_joint", "right_thumb_2_joint"]
                    }},
                    {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
                    {"use_sim_time": True},
                    ompl_planning_pipeline_config
                ],
            )
        ]
    )


    return LaunchDescription(
        [
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller,
            hand_controller,
            move_group_node,
        ]
    )

if __name__ == '__main__':
    generate_launch_description()
