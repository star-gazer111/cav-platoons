# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, ExecuteProcess, GroupAction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node

# def generate_launch_description():
#     pkg_path = get_package_share_directory('paper_implementation')
#     turtlebot_model = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf'
    
#     # 1. Start Gazebo
#     world_cmd = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
#         ]),
#     )

#     # 2. Spawn Obstacle (Red Box) at x=3.0
#     spawn_obstacle = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-entity', 'paper_obstacle', 
#                    '-file', os.path.join(pkg_path, 'models', 'obstacle.sdf'),
#                    '-x', '1.0', '-y', '0.0', '-z', '0.175'],
#         output='screen'
#     )

#     # Make the obstacle move forward at a constant 0.1 m/s
#     move_obstacle_cmd = ExecuteProcess(
#         cmd=['ros2', 'topic', 'pub', '/obstacle/cmd_vel', 'geometry_msgs/msg/Twist', 
#              '{linear: {x: 0.06, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
#         output='screen'
#     )

#     # --- ROBOT SPAWNING FUNCTION ---
#     def create_robot_group(name, x_pos):
#         return GroupAction([
#             # Spawn the robot model in Gazebo
#             Node(
#                 package='gazebo_ros',
#                 executable='spawn_entity.py',
#                 arguments=['-entity', name, 
#                            '-file', turtlebot_model,
#                            '-x', str(x_pos), '-y', '0.0', '-z', '0.01',
#                            '-robot_namespace', name],
#                 output='screen'
#             ),
#             # Start the Brain (Collision Node) inside the namespace
#             Node(
#                 package='paper_implementation',
#                 executable='paper_node',
#                 namespace=name,
#                 parameters=[{'frame_prefix': name}], 
#                 output='screen'
#             )
#         ])

#     # 3. Create Platoon (Leader -> Follower 1 -> Follower 2)
#     robot_0 = create_robot_group('tb3_0', 0.0)   # Leader
#     robot_1 = create_robot_group('tb3_1', -0.5)  # Follower 1 
#     robot_2 = create_robot_group('tb3_2', -1.0)  # Follower 2 

#     return LaunchDescription([
#         world_cmd,
#         spawn_obstacle,
#         move_obstacle_cmd,
#         robot_0,
#         robot_1,
#         robot_2
#     ])


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('paper_implementation')
    turtlebot_model = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf'
    
    # 1. Start Gazebo
    world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
    )

    # 2a. Spawn Obstacle 1 at x=3.0, y=0.0
    spawn_obstacle_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'paper_obstacle_1', 
                   '-file', os.path.join(pkg_path, 'models', 'obstacle.sdf'),
                   '-x', '3.0', '-y', '0.0', '-z', '0.175',
                   '-robot_namespace', 'obstacle_1'], # Added namespace to isolate cmd_vel
        output='screen'
    )

    # 2b. Spawn Obstacle 2 at x=3.0, y=-0.65
    spawn_obstacle_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'paper_obstacle_2', 
                   '-file', os.path.join(pkg_path, 'models', 'obstacle.sdf'),
                   '-x', '3.0', '-y', '-0.65', '-z', '0.175',
                   '-robot_namespace', 'obstacle_2'], # Added namespace to isolate cmd_vel
        output='screen'
    )

    # Make Obstacle 1 move forward at a constant 0.06 m/s
    move_obstacle_1_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/obstacle_1/cmd_vel', 'geometry_msgs/msg/Twist', 
             '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
        output='screen'
    )

    # Make Obstacle 2 move forward at a constant 0.06 m/s
    move_obstacle_2_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/obstacle_2/cmd_vel', 'geometry_msgs/msg/Twist', 
             '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
        output='screen'
    )

    # --- ROBOT SPAWNING FUNCTION ---
    def create_robot_group(name, x_pos):
        return GroupAction([
            # Spawn the robot model in Gazebo
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', name, 
                           '-file', turtlebot_model,
                           '-x', str(x_pos), '-y', '0.0', '-z', '0.01',
                           '-robot_namespace', name],
                output='screen'
            ),
            # Start the Brain (Collision Node) inside the namespace
            Node(
                package='paper_implementation',
                executable='paper_node',
                namespace=name,
                parameters=[{'frame_prefix': name}], 
                output='screen'
            )
        ])

    # 3. Create Platoon (Leader -> Follower 1 -> Follower 2)
    robot_0 = create_robot_group('tb3_0', 0.0)   # Leader
    robot_1 = create_robot_group('tb3_1', -0.5)  # Follower 1 
    robot_2 = create_robot_group('tb3_2', -1.0)  # Follower 2 

    return LaunchDescription([
        world_cmd,
        spawn_obstacle_1,
        spawn_obstacle_2,
        move_obstacle_1_cmd,
        move_obstacle_2_cmd,
        robot_0,
        robot_1,
        robot_2
    ])