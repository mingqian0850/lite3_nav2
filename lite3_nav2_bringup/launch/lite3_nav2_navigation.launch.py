from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Minimal Nav2 bringup for Lite3:
    - 启动 controller_server + bt_navigator + behavior_server + lifecycle_manager
    - 假设上层程序自己提供路径（FollowPath），外部 SLAM 提供 TF
    """

    bringup_dir = get_package_share_directory("lite3_nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    default_bt_xml = LaunchConfiguration("default_bt_xml")

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

    declare_default_bt_xml = DeclareLaunchArgument(
        "default_bt_xml",
        default_value=PathJoinSubstitution(
            [bringup_dir, "params", "follow_path_wait_from_topic.xml"]
        ),
        description="Behavior tree XML to load for bt_navigator.",
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "default_bt_xml_filename": default_bt_xml,
                "default_nav_to_pose_bt_xml": default_bt_xml,
                "default_nav_through_poses_bt_xml": default_bt_xml,
            },
        ],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
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
            {"node_names": ["controller_server", "bt_navigator", "behavior_server"]},
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_params_file,
            declare_autostart,
            declare_default_bt_xml,
            controller_server,
            bt_navigator,
            behavior_server,
            lifecycle_manager,
        ]
    )


