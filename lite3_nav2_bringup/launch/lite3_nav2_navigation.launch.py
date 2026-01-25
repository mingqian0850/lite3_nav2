from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
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
        default_value="true",
        description="Use simulation (Gazebo) clock if true.",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [bringup_dir, "params", "nav2_params_v1.yaml"]
        ),
        description="Full path to the ROS2 parameters file to use for Nav2.",
    )

    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the Nav2 stack.",
    )

    declare_waypoints_topic = DeclareLaunchArgument(
        "input_waypoints_topic",
        default_value="/waypoints/raw",
        description="Input Path topic (nav_msgs/Path).",
    )
    declare_interpolated_vis_topic = DeclareLaunchArgument(
        "interpolated_waypoints_visualization_topic",
        default_value="/waypoints/interpolated/markers",
        description="Output marker topic for interpolated waypoints visualization.",
    )
    declare_interpolated_path_topic = DeclareLaunchArgument(
        "interpolated_waypoints_topic",
        default_value="/waypoints/interpolated/path",
        description="Output Path topic for interpolated waypoints.",
    )
    declare_interpolation_step = DeclareLaunchArgument(
        "interpolation_step",
        default_value="0.1",
        description="Interpolation step (meters).",
    )
    declare_send_retries = DeclareLaunchArgument(
        "send_retries",
        default_value="10",
        description="Retry count on ABORTED for send_follow_path.",
    )
    declare_send_retry_wait = DeclareLaunchArgument(
        "send_retry_wait",
        default_value="0.5",
        description="Retry wait seconds for send_follow_path.",
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    send_follow_path = Node(
        package="lite3_nav2_bringup",
        executable="send_follow_path",
        name="send_follow_path",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--waypoints-topic",
            LaunchConfiguration("input_waypoints_topic"),
            "--step",
            LaunchConfiguration("interpolation_step"),
            "--retries",
            LaunchConfiguration("send_retries"),
            "--retry-wait",
            LaunchConfiguration("send_retry_wait"),
        ],
    )

    visualize_waypoints = Node(
        package="lite3_nav2_bringup",
        executable="visualize_waypoints",
        name="visualize_waypoints",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "--waypoints-topic",
            LaunchConfiguration("input_waypoints_topic"),
            "--topic",
            LaunchConfiguration("interpolated_waypoints_visualization_topic"),
            "--step",
            LaunchConfiguration("interpolation_step"),
        ],
    )

    start_after_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_server,
            on_start=[send_follow_path, visualize_waypoints],
        )
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
            declare_waypoints_topic,
            declare_interpolated_vis_topic,
            declare_interpolated_path_topic,
            declare_interpolation_step,
            declare_send_retries,
            declare_send_retry_wait,
            controller_server,
            start_after_controller,
            lifecycle_manager,
        ]
    )


