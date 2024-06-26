from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    semitrailer_simulation_share_dir = FindPackageShare("semitrailer_simulation")
    semitrailer_description_share_dir = FindPackageShare("semitrailer_description")

    semitrailer_xacro = PathJoinSubstitution(
        [semitrailer_description_share_dir, "urdf", "semitrailer.urdf"]
    )
    semitrailer_props_yaml = PathJoinSubstitution(
        [semitrailer_simulation_share_dir, "config", "semitrailer_40.yaml"]
    )
    joint_state_publisher_params_yaml = PathJoinSubstitution(
        [semitrailer_simulation_share_dir, "config", "joint_state_publisher.yaml"]
    )
    semitrailer_simulator_params_yaml = PathJoinSubstitution(
        [semitrailer_simulation_share_dir, "config", "semitrailer_simulator.yaml"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        semitrailer_xacro,
                        " props_yaml:=",
                        semitrailer_props_yaml,
                    ]
                )
            }
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[joint_state_publisher_params_yaml],
    )

    semitrailer_simulator_node = Node(
        package="semitrailer_simulation",
        executable="semitrailer_simulator_node",
        name="semitrailer_simulator",
        output="screen",
        parameters=[semitrailer_props_yaml, semitrailer_simulator_params_yaml],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [semitrailer_simulation_share_dir, "rviz", "default.rviz"]
            ),
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_publisher_node,
            semitrailer_simulator_node,
            rviz2_node,
        ]
    )
