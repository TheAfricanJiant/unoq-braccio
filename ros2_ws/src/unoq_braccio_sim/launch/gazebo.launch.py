import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory("unoq_braccio_sim")
    xacro_path = os.path.join(share, "urdf", "braccio.urdf.xacro")
    world_path = os.path.join(share, "worlds", "workspace.world")
    controllers_path = os.path.join(share, "config", "controllers.yaml")
    mesh_dir = os.path.join(share, "meshes", "braccio_stedden")
    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                xacro_path,
                " controllers_file:=",
                controllers_path,
                " mesh_dir:=",
                mesh_dir,
            ]
        ),
        "use_sim_time": True,
    }
    sim_time = {"use_sim_time": True}

    fallback_sim = DeclareLaunchArgument(
        "fallback_sim", default_value="false",
        description="Also run joint_state_simulator. Only for debugging without "
        "controllers: it publishes /joint_states and fights the broadcaster.",
    )
    detector = DeclareLaunchArgument(
        "detector", default_value="true",
        description="Start sim_cube_detector on the overhead camera.",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": ["-r ", world_path]}.items(),
    )

    # Gazebo -> ROS: sim clock and both camera streams. Without /clock every
    # use_sim_time node (controllers included) stays frozen at t=0.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/overhead_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/gripper_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/gripper_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            ("/overhead_camera/image_raw", "/vision/overhead/image_raw"),
            ("/overhead_camera/camera_info", "/vision/overhead/camera_info"),
            ("/gripper_camera/image_raw", "/vision/gripper/image_raw"),
            ("/gripper_camera/camera_info", "/vision/gripper/camera_info"),
        ],
        parameters=[sim_time],
        output="screen",
    )

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "unoq_braccio", "-topic", "robot_description",
                   "-x", "0", "-y", "0", "-z", "0"],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
        parameters=[sim_time],
        output="screen",
    )
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
        parameters=[sim_time],
        output="screen",
    )

    trajectory_bridge = Node(
        package="unoq_braccio_driver",
        executable="joint_trajectory_bridge",
        name="unoq_braccio_joint_trajectory_bridge",
        parameters=[sim_time],
        output="screen",
    )
    joint_state_simulator = Node(
        package="unoq_braccio_driver",
        executable="joint_state_simulator",
        name="unoq_braccio_joint_state_simulator",
        parameters=[sim_time],
        condition=IfCondition(LaunchConfiguration("fallback_sim")),
        output="screen",
    )
    cube_detector = Node(
        package="unoq_braccio_driver",
        executable="sim_cube_detector",
        name="sim_cube_detector",
        parameters=[sim_time],
        condition=IfCondition(LaunchConfiguration("detector")),
        output="screen",
    )

    return LaunchDescription(
        [
            fallback_sim,
            detector,
            gz_sim,
            bridge,
            state_publisher,
            spawn,
            # Controllers are started in order, each once the previous one exits,
            # instead of on a fixed timer that races Gazebo start-up.
            RegisterEventHandler(
                OnProcessExit(target_action=spawn, on_exit=[joint_state_broadcaster])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=joint_state_broadcaster, on_exit=[arm_controller])
            ),
            trajectory_bridge,
            joint_state_simulator,
            cube_detector,
        ]
    )
