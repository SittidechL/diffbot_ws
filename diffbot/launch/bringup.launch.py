from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # ✅ robot_description (xacro → urdf)
    robot_description = {
        "robot_description": Command([
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare("diffbot_description"),
                "urdf",
                "diffbot.urdf.xacro"
            ])
        ])
    }

    # ✅ robot_state_publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # ✅ ros2_control_node (controller_manager)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([
                FindPackageShare("diffbot"),
                "config",
                "controller.yaml"
            ])
        ],
        remappings=[
            ("/diff_drive_controller/odom", "/odom"),
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
        output="screen",
    )


      # ✅ joint_state_broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager"
        ],
        output="screen",
    )

    # ✅ diff_drive_controller
    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager", "/controller_manager"
        ],
        output="screen",
    )

    return LaunchDescription([
        rsp,
        ros2_control_node,
        joint_state_broadcaster,
        diff_drive_controller,
    ])
