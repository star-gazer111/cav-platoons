# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node

# def generate_launch_description():
#     pkg_paper = get_package_share_directory('paper_implementation')
#     pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
#     pkg_turtlebot3 = get_package_share_directory('turtlebot3_gazebo')

#     # 1. Start Gazebo (Empty World)
#     # 1. Start Gazebo (Standard ROS 2 way)
#     world_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#                 os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
#             ]),
#         launch_arguments={'pause': 'false'}.items()
#         )

#     # 2. Spawn Obstacle (The Red Box) at x=3.0, y=0.05
#     # We read the SDF file we created
#     obstacle_sdf_path = os.path.join(
#         os.getcwd(), 'src', 'paper_implementation', 'models', 'obstacle.sdf'
#     )

#     spawn_obstacle = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=[
#             '-entity', 'paper_obstacle',
#             '-file', obstacle_sdf_path,
#             '-x', '3.0',
#             '-y', '0.05',
#             '-z', '0.0'
#         ],
#         output='screen'
#     )

#     # 3. Spawn TurtleBot3 at x=0.0, y=0.0
#     # We reuse the official TurtleBot3 spawn launch
#     spawn_turtlebot = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_turtlebot3, 'launch', 'spawn_turtlebot3.launch.py')
#         ),
#         launch_arguments={
#             'x_pose': '0.0',
#             'y_pose': '0.0'
#         }.items()
#     )

#     # 4. (Optional) Run your Collision Avoidance Node automatically
#     # Uncomment the lines below if you want the algorithm to start with the simulation
#     # run_algo = Node(
#     #     package='paper_implementation',
#     #     executable='paper_node',
#     #     output='screen'
#     # )

#     return LaunchDescription([
#         world_cmd,
#         spawn_obstacle,
#         spawn_turtlebot,
#         # run_algo
#     ])


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('paper_implementation')
    
    # 1. Start Gazebo
    world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
    )

    # 2. Spawn TurtleBot (The Agent)
    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'burger', 
                   '-file', '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf',
                   '-x', '0.0', '-y', '0.0', '-z', '0.01'],
        output='screen'
    )

    # 3. Spawn Obstacle (The Moving Box)
    spawn_obstacle = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'paper_obstacle', 
                   '-file', os.path.join(pkg_path, 'models', 'obstacle.sdf'),
                   '-x', '3.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # 4. Move the Obstacle (Velocity Command)
    move_obstacle_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/obstacle/cmd_vel', 'geometry_msgs/msg/Twist', 
             '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
        output='screen'
    )

    # 5. START YOUR NODE (The Brain)
    # This runs the paper_node automatically
    run_paper_node = Node(
        package='paper_implementation',
        executable='paper_node',
        output='screen'
    )

    # 6. Delay the brain start
    # We wait until the robot is spawned before starting the logic
    # otherwise it might try to read Lidar before the robot exists.
    delayed_node_start = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_turtlebot,
            on_exit=[run_paper_node],
        )
    )

    return LaunchDescription([
        world_cmd,
        spawn_turtlebot,
        spawn_obstacle,
        move_obstacle_cmd,
        delayed_node_start  # Contains the logic to run paper_node
    ])