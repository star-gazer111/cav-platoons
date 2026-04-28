           
                                                                     
                                      
                                                                            
                                                                             
                                                      
                                     

                                    
                                                                     
                                                                
                                                                       

                                     
                                            
                                           
                                         
                                                                                                       
                 
                                                     
           

                                                        
                                       
                                       
                                                                              
       

                            
                               
                                       
                     
                                          
                                         
                          
                           
                         
            
                         
       

                                           
                                                     
                                                 
                                        
                                                                                  
            
                            
                              
                             
                   
       

                                                                     
                                                                                        
                        
                                           
                                    
                           
         

                                
                    
                         
                          
                    
        


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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

                                    
    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'burger', 
                   '-file', turtlebot_model,
                   '-x', '0.0', '-y', '0.0', '-z', '0.01'],
        output='screen'
    )

                                        
    spawn_obstacle = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'paper_obstacle', 
                   '-file', os.path.join(pkg_path, 'models', 'obstacle.sdf'),
                   '-x', '3.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

                                             
    move_obstacle_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/obstacle/cmd_vel', 'geometry_msgs/msg/Twist', 
             '{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
        output='screen'
    )

                                    
                                                                                 
    run_paper_node = Node(
        package='paper_implementation',
        executable='collision_node_onesv_onemoving_ov',
        output='screen'
    )

                              
                                                                  
                                                                   
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
        delayed_node_start
    ])
