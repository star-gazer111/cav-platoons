from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    common_params = {
        "use_sim_time": False,
        "leader_speed": 0.18,
        "desired_headway": 1.0,
        "headway_gain": 1.2,
        "follower_path_preview": 0.30,
        "lane_change_duration": 2.2,
        "avoid_trigger_distance": 1.4,
        "avoid_full_ahead": 1.0,
        "clear_distance_behind": 1.10,
        "obstacle_speed_margin": 0.06,
        "follower_obstacle_buffer": 0.40,
        "ov_prediction_time": 1.20,
        "return_prediction_time": 1.00,
        "tail_clear_margin": 0.45,
        "leader_clear_margin": 0.75,
        "min_relative_speed": 0.02,
        "max_scan_range": 3.50,
        "startup_gap_tolerance": 0.08,
        "startup_creep_speed": 0.08,
        "preferred_pass_side": "right",
        "enable_debug_logs": True,
        "debug_interval_sec": 1.0,
        "wait_warn_interval_sec": 2.0,
        "use_initial_pose_offsets": LaunchConfiguration("use_initial_pose_offsets"),
        "sv0_initial_x": LaunchConfiguration("sv0_initial_x"),
        "sv0_initial_y": LaunchConfiguration("sv0_initial_y"),
        "sv1_initial_x": LaunchConfiguration("sv1_initial_x"),
        "sv1_initial_y": LaunchConfiguration("sv1_initial_y"),
        "ov_initial_x": LaunchConfiguration("ov_initial_x"),
        "ov_initial_y": LaunchConfiguration("ov_initial_y"),
        "sv0_odom_topic": LaunchConfiguration("sv0_odom_topic"),
        "sv1_odom_topic": LaunchConfiguration("sv1_odom_topic"),
        "ov_odom_topic": LaunchConfiguration("ov_odom_topic"),
        "nominal_obstacle_speed": LaunchConfiguration("ov_linear_speed"),
    }

    leader_node = Node(
        package="paper_implementation",
        executable="collision_node_two_sv_one_moving_ov",
        namespace="tb3_0",
        name="two_sv_one_moving_ov",
        parameters=[
            common_params,
            {
                "bot_id": 0,
                "cmd_vel_topic": LaunchConfiguration("leader_cmd_vel_topic"),
            },
        ],
        output="screen",
    )

    follower_node = Node(
        package="paper_implementation",
        executable="collision_node_two_sv_one_moving_ov",
        namespace="tb3_1",
        name="two_sv_one_moving_ov",
        parameters=[
            common_params,
            {
                "bot_id": 1,
                "cmd_vel_topic": LaunchConfiguration("follower_cmd_vel_topic"),
            },
        ],
        output="screen",
    )

    moving_ov_driver = Node(
        package="paper_implementation",
        executable="moving_obstacle_straight_driver",
        name="moving_obstacle_straight_driver",
        parameters=[
            {
                "cmd_vel_topic": LaunchConfiguration("ov_cmd_vel_topic"),
                "linear_speed": LaunchConfiguration("ov_linear_speed"),
                "angular_speed": LaunchConfiguration("ov_angular_speed"),
                "enable_debug_logs": True,
                "log_interval_sec": 2.0,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("sv0_odom_topic", default_value="/tb3_0/odom"),
            DeclareLaunchArgument("sv1_odom_topic", default_value="/tb3_1/odom"),
            DeclareLaunchArgument("ov_odom_topic", default_value="/tb3_2/odom"),
            DeclareLaunchArgument("leader_cmd_vel_topic", default_value="cmd_vel"),
            DeclareLaunchArgument("follower_cmd_vel_topic", default_value="cmd_vel"),
            DeclareLaunchArgument("ov_cmd_vel_topic", default_value="/tb3_2/cmd_vel"),
            DeclareLaunchArgument("use_initial_pose_offsets", default_value="true"),
            DeclareLaunchArgument("sv0_initial_x", default_value="0.0"),
            DeclareLaunchArgument("sv0_initial_y", default_value="0.0"),
            DeclareLaunchArgument("sv1_initial_x", default_value="-0.5"),
            DeclareLaunchArgument("sv1_initial_y", default_value="0.0"),
            DeclareLaunchArgument("ov_initial_x", default_value="3.0"),
            DeclareLaunchArgument("ov_initial_y", default_value="0.0"),
            DeclareLaunchArgument("ov_linear_speed", default_value="0.08"),
            DeclareLaunchArgument("ov_angular_speed", default_value="0.0"),
            leader_node,
            follower_node,
            moving_ov_driver,
        ]
    )
