from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    semitrailer_simulation_share_dir = FindPackageShare("semitrailer_simulation")

    semitrailer_props_yaml = PathJoinSubstitution(
        [semitrailer_simulation_share_dir, "config", "semitrailer_40.yaml"]
    )
    simple_semitrailer_navigation_params_yaml = PathJoinSubstitution(
        [
            semitrailer_simulation_share_dir,
            "config",
            "simple_semitrailer_navigation.yaml",
        ]
    )

    bringup_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [semitrailer_simulation_share_dir, "launch", "bringup.launch.py"]
        )
    )

    simple_semitrailer_navigation_node = Node(
        package="semitrailer_controller",
        executable="simple_semitrailer_navigation_node",
        name="simple_semitrailer_navigation",
        output="screen",
        remappings={
            "~/state": "/semitrailer_simulator/state",
            "~/input": "/semitrailer_simulator/input",
        }.items(),
        parameters=[semitrailer_props_yaml, simple_semitrailer_navigation_params_yaml],
    )

    return LaunchDescription(
        [
            bringup_launch,
            simple_semitrailer_navigation_node,
        ]
    )
