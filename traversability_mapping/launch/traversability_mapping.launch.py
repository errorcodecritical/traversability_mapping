from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    parameters = PathJoinSubstitution([
        FindPackageShare("traversability_mapping"),
        "config",
        "traversability_mapping.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="traversability_mapping",
            executable="traversability_filter",
            name="traversability_filter",
            output="screen",
            parameters=[parameters, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="traversability_mapping",
            executable="traversability_map",
            name="traversability_map",
            output="screen",
            parameters=[parameters, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="traversability_mapping",
            executable="traversability_prm",
            name="traversability_prm",
            output="screen",
            parameters=[parameters, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="traversability_mapping",
            executable="traversability_path",
            name="traversability_path",
            output="screen",
            parameters=[parameters, {"use_sim_time": use_sim_time}],
        ),
    ])
