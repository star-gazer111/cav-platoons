           
                                                                     
                                      
                                                                                  
                                                                             
                                     

                                    
                                                                    
                                                                                                    
    
                       
                                           
                                         
                                                                                                   
             
       

                                            
                            
                               
                                       
                                                  
                                                                               
                                                              
                         
       

                                                            
                                         
                                                                                       
                                                                                         
                         
       

                                       
                                          
                              
                                               
                   
                                       
                                               
                                              
                                                      
                                                                         
                                                       
                                 
                
                                                                     
                   
                                                 
                                          
                                 
                                                       
                                 
               
            

                                                              
                                                           
                                                                
                                                                

                                
                    
                         
                            
                  
                  
                 
        


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('paper_implementation')
    turtlebot_model = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_burger',
        'model.sdf',
    )
    
                     
    world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
    )

                                          
    spawn_obstacle_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'paper_obstacle_1', 
                   '-file', os.path.join(pkg_path, 'models', 'obstacle.sdf'),
                   '-x', '3.0', '-y', '0.0', '-z', '0.175',
                   '-robot_namespace', 'obstacle_1'],                                     
        output='screen'
    )

                                            
    spawn_obstacle_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'paper_obstacle_2', 
                   '-file', os.path.join(pkg_path, 'models', 'obstacle.sdf'),
                   '-x', '3.0', '-y', '-0.65', '-z', '0.175',
                   '-robot_namespace', 'obstacle_2'],                                     
        output='screen'
    )

                                                         
    move_obstacle_1_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/obstacle_1/cmd_vel', 'geometry_msgs/msg/Twist', 
             '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
        output='screen'
    )

                                                         
    move_obstacle_2_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/obstacle_2/cmd_vel', 'geometry_msgs/msg/Twist', 
             '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
        output='screen'
    )

                                     
    def create_robot_group(name, x_pos):
        return GroupAction([
                                             
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', name, 
                           '-file', turtlebot_model,
                           '-x', str(x_pos), '-y', '0.0', '-z', '0.01',
                           '-robot_namespace', name],
                output='screen'
            ),
                                                                   
            Node(
                package='paper_implementation',
                executable='paper_node',
                namespace=name,
                parameters=[{'frame_prefix': name}], 
                output='screen'
            )
        ])

                                                            
    robot_0 = create_robot_group('tb3_0', 0.0)           
    robot_1 = create_robot_group('tb3_1', -0.5)               
    robot_2 = create_robot_group('tb3_2', -1.0)               

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
