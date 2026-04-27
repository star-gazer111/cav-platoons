import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def create_spawn_robot(robot_name, x_pos, turtlebot_model):
    return Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            robot_name,
            "-file",
            turtlebot_model,
            "-x",
            str(x_pos),
            "-y",
            "0.0",
            "-z",
            "0.01",
            "-timeout",
            "120.0",
            "-robot_namespace",
            robot_name,
        ],
        output="screen",
    )


def create_planner(robot_name, planner_params):
    return Node(
        package="paper_implementation",
        executable="collision_node_two_sv_one_stationary_ov",
        namespace=robot_name,
        parameters=[planner_params],
        output="screen",
    )


def generate_launch_description():
    pkg_path = get_package_share_directory("paper_implementation")
    turtlebot_model = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "models",
        "turtlebot3_burger",
        "model.sdf",
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gzserver.launch.py",
                )
            ]
        ),
        launch_arguments={
            "verbose": "true",
            "factory": "true",
            "server_required": "true",
        }.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gzclient.launch.py",
                )
            ]
        ),
    )

    spawn_obstacle = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "paper_obstacle",
            "-file",
            os.path.join(pkg_path, "models", "obstacle.sdf"),
            "-x",
            "3.0",
            "-y",
            "0.0",
            "-z",
            "0.175",
            "-timeout",
            "120.0",
        ],
        output="screen",
    )

    planner_params = {
        "leader_speed": 0.14,
        "desired_headway": 0.50,
        "headway_gain": 0.90,
        "lane_change_duration": 2.2,
        "avoid_trigger_distance": 1.4,
        "clear_distance_behind": 0.9,
        "obstacle_initial_x": 3.0,
        "obstacle_initial_y": 0.0,
        "preferred_pass_side": "right",
    }

    spawn_tb3_0 = create_spawn_robot("tb3_0", 0.0, turtlebot_model)
    spawn_tb3_1 = create_spawn_robot("tb3_1", -0.5, turtlebot_model)

    run_tb3_0_planner = create_planner("tb3_0", planner_params)
    run_tb3_1_planner = create_planner("tb3_1", planner_params)

    delayed_spawns = TimerAction(
        period=5.0,
        actions=[spawn_obstacle, spawn_tb3_0, spawn_tb3_1],
    )

    delayed_planner_start = TimerAction(
        period=9.0,
        actions=[run_tb3_0_planner, run_tb3_1_planner],
    )

    return LaunchDescription(
        [
            gzserver_cmd,
            gzclient_cmd,
            delayed_spawns,
            delayed_planner_start,
        ]
    )
