import os
import yaml
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import xacro
from moveit_configs_utils import MoveItConfigsBuilder
from launch.conditions import IfCondition
from launch.substitutions import TextSubstitution


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    xacro_griper_select = LaunchConfiguration("long_dist_gripper")

    declare_xacro_gripper_select = DeclareLaunchArgument(
        "long_dist_gripper",
        default_value="true",
        description="Select the gripper type to use",
    )

    # Manually process the xacro file
    xacro_file = os.path.join(
        get_package_share_directory("arm_moveit_config"), "config", "arm.urdf.xacro"
    )
    robot_description_config = xacro.process_file(
        xacro_file, mappings={"long_dist_gripper": "false"}
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    moveit_config = MoveItConfigsBuilder("arm").to_moveit_configs()
    moveit_config.robot_description = robot_description

    # Get parameters for the Servo node
    servo_yaml = load_yaml("moveit_servo", "config/arm_simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # RViz
    rviz_config_file = (
        get_package_share_directory("arm_moveit_config") + "/config/moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("arm_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager",],
    )
    
    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "velocity_controller",
            "--controller-manager",
            "/controller_manager",
            "--inactive",
        ],
    )
    
    chassis_transform_spawner = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='chassis_to_arm_tf',
        arguments=[
            '-0.14', '-0.153', '0.17',   # translation: x y z
            '0', '0', '0',         # rotation: roll pitch yaw
            'chassis_link',              # parent frame
            'arm_base_link'              # child frame
        ]
    )

    # Start the actual move_group node/action server (added from move_group)
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoServer",
            #     name="servo_server",
            #     parameters=[
            #         servo_params,
            #         moveit_config.robot_description,
            #         moveit_config.robot_description_semantic,
            #     ],
            # ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
        # Add log level
        # arguments=["--ros-args", "--log-level", "debug"]
    )

    delayed_moveit_nodes = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[
            chassis_transform_spawner,
        ],
    )
    
    joy_to_twist_node = Node(
        package="rover_arm_scripts",  # Replace with the correct package name if different
        executable="joy_to_twist",
        name="joy_to_twist",
        output="screen",
    )
    
    joy_repeater_node = Node(
        package="rover_arm_scripts",
        executable="joy_repeater",
        name="joy_repeater",
        output="screen",
    )

    nodes = [
        declare_xacro_gripper_select,
        # delayed_moveit_nodes,
        joint_state_broadcaster_spawner,
        rviz_node,
        servo_node,
        container,
        arm_controller_spawner,
        ros2_control_node,
        run_move_group_node,    # Added
        # velocity_controller_spawner,
        joy_to_twist_node,  
        joy_repeater_node,
        chassis_transform_spawner,
    ]

    return LaunchDescription(nodes)
