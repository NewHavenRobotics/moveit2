import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None
    
def generate_launch_description():
    # Load additional robot description and planning configurations
    robot_description_config = load_file(
        "arm_moveit_config", "config/arm.urdf.xacro"
    )
    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config = load_file(
        "arm_moveit_config", "config/arm.srdf"
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}

    kinematics_yaml = load_yaml(
        "arm_moveit_config", "config/kinematics.yaml"
    )

    planning_yaml = load_yaml(
        "arm_moveit_config", "config/ompl_planning.yaml"
    )

    planning_plugin = {"planning_plugin": "ompl_interface/OMPLPlanner"}
    
    return LaunchDescription(
        [
            Node(
                package="moveit_servo",
                executable="keyboard_autonomy_node",
                name="keyboard_autonomy_node",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    planning_yaml,
                    planning_plugin,
                ],
            )
        ]
    )