import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    is_sim = LaunchConfiguration("is_sim")
    is_ignition = LaunchConfiguration("is_ignition")

    ros_distro = os.environ.get("ROS_DISTRO", "").lower()
    default_is_ignition = "True" if ros_distro == "humble" else "False"
    
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    is_ignition_arg = DeclareLaunchArgument(
        "is_ignition",
        default_value=default_is_ignition
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("panda_description"),
                    "urdf",
                    "panda.urdf.xacro",
                ),
                " is_sim:=",
                is_sim,
                " is_ignition:=",
                is_ignition
            ]
        ),
        value_type=str
    )

    debug_info = LogInfo(
        msg=[
            "[panda_controller] ros_distro=",
            ros_distro,
            " is_ignition=",
            is_ignition,
            " robot_description xacro: ",
            os.path.join(
                get_package_share_directory("panda_description"),
                "urdf",
                "panda.urdf.xacro",
            ),
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": is_sim}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        arguments=[
            "--ros-args",
            "--log-level",
            "controller_manager:=debug",
            "--log-level",
            "hardware_interface:=debug",
        ],
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": is_sim},
            os.path.join(
                get_package_share_directory("panda_controller"),
                "config",
                "panda_controllers.yaml",
            ),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": is_sim}],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": is_sim}],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": is_sim}],
    )

    return LaunchDescription(
        [
            is_sim_arg,
            is_ignition_arg,
            debug_info,
            robot_state_publisher_node,
            controller_manager,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
        ]
    )
