from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    common_params = {
        "use_sim_time": False,
        "leader_speed": 0.14,
        "desired_headway": 0.70,
        "headway_gain": 0.90,
        "follower_path_preview": 0.35,
        "lane_change_duration": 2.2,
        "avoid_trigger_distance": 2.0,
        "clear_distance_behind": 0.9,
        "preferred_pass_side": "right",
        "enable_debug_logs": True,
        "debug_interval_sec": 1.0,
        "wait_warn_interval_sec": 2.0,
                                                                    
                                                                          
                                                                           
        "use_initial_pose_offsets": LaunchConfiguration("use_initial_pose_offsets"),
        "sv0_initial_x": LaunchConfiguration("sv0_initial_x"),
        "sv0_initial_y": LaunchConfiguration("sv0_initial_y"),
        "sv1_initial_x": LaunchConfiguration("sv1_initial_x"),
        "sv1_initial_y": LaunchConfiguration("sv1_initial_y"),
        "sv0_odom_topic": LaunchConfiguration("sv0_odom_topic"),
        "sv1_odom_topic": LaunchConfiguration("sv1_odom_topic"),
    }

    leader_node = Node(
        package="paper_implementation",
        executable="collision_node_two_sv_one_stationary_ov",
        namespace="tb3_0",
        name="two_sv_one_stationary_ov",
        parameters=[
            common_params,
            {
                "bot_id": 0,
                "scan_topic": LaunchConfiguration("leader_scan_topic"),
                "cmd_vel_topic": LaunchConfiguration("leader_cmd_vel_topic"),
            },
        ],
        output="screen",
    )

    follower_node = Node(
        package="paper_implementation",
        executable="collision_node_two_sv_one_stationary_ov",
        namespace="tb3_1",
        name="two_sv_one_stationary_ov",
        parameters=[
            common_params,
            {
                "bot_id": 1,
                "cmd_vel_topic": LaunchConfiguration("follower_cmd_vel_topic"),
            },
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("sv0_odom_topic", default_value="/tb3_0/odom"),
            DeclareLaunchArgument("sv1_odom_topic", default_value="/tb3_1/odom"),
            DeclareLaunchArgument("leader_scan_topic", default_value="scan"),
            DeclareLaunchArgument("leader_cmd_vel_topic", default_value="cmd_vel"),
            DeclareLaunchArgument("follower_cmd_vel_topic", default_value="cmd_vel"),
            DeclareLaunchArgument("use_initial_pose_offsets", default_value="true"),
            DeclareLaunchArgument("sv0_initial_x", default_value="0.0"),
            DeclareLaunchArgument("sv0_initial_y", default_value="0.0"),
            DeclareLaunchArgument("sv1_initial_x", default_value="-0.5"),
            DeclareLaunchArgument("sv1_initial_y", default_value="0.0"),
            leader_node,
            follower_node,
        ]
    )
