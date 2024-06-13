from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # RViz configuration files
    rviz_config_1 = DeclareLaunchArgument(
        'rviz_config_1',
        default_value='/home/nataraj/.rviz2/project5_view.rviz',
        description='Path to RViz config for rviz2_1'
    )

    # Launch rviz2_1
    rviz2_node_1 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_1',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_1')]  # Load RViz config file
    )

    # Run nodes
    sim1 = Node(
        package='planning',
        executable='sim1',
        name='robot',
        output='screen'
    )
    
    togoal = Node(
        package='planning',
        executable='togoal',
        name='MoveRobot',
        output='screen'
    )

    return LaunchDescription([
        rviz_config_1,
        rviz2_node_1,
        sim1,
        togoal,
    ])

    
    """
    # Record a bag file
    bag_record_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'record','/scan','/pose','/occupancy_grid','/robot_description','/tf','/environment', '-o', '03-16'],
        output='screen'
    )
    
        
    # Ensure the bag record process is terminated when bag play finishes
    bag_play_exit_event = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play_node,
            on_exit=[
                ExecuteProcess(cmd=['pkill', '-f', 'ros2 bag record'])
            ]
        )
    )
    """
    
