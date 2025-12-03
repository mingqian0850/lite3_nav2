from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Minimal Nav2 bringup for Lite3:
    - 仅启动 controller_server + local_costmap + lifecycle_manager
    - 假设上层程序自己提供路径（FollowPath），外部 SLAM 提供 TF
    """

    bringup_dir = get_package_share_directory("lite3_nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true.",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [bringup_dir, "params", "nav2_params.yaml"]
        ),
        description="Full path to the ROS2 parameters file to use for Nav2.",
    )

    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the Nav2 stack.",
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
                {"node_names": ["controller_server"]},
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_params_file,
            declare_autostart,
            controller_server,
            lifecycle_manager,
        ]
    )


