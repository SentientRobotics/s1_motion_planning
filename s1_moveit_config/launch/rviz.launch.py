from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

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

    rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                output="both",
                arguments=["-d", os.path.join(get_package_share_directory("s1_moveit_config"), "config", "moveit.rviz")],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                ],
            )


    return LaunchDescription([
        rviz_node,
    ])

if __name__ == '__main__':
    generate_launch_description()
