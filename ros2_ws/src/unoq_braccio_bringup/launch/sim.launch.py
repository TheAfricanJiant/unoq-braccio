from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_launch = PathJoinSubstitution(
        [FindPackageShare("unoq_braccio_sim"), "launch", "gazebo.launch.py"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("fallback_sim", default_value="false"),
            DeclareLaunchArgument("detector", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    "fallback_sim": LaunchConfiguration("fallback_sim"),
                    "detector": LaunchConfiguration("detector"),
                    "rviz": LaunchConfiguration("rviz"),
                }.items(),
            ),
        ]
    )
