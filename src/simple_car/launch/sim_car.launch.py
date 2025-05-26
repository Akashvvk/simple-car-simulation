import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Get the URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('simple_car'),
        'urdf',
        'simple_car.urdf'
    )

    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Get the Gazebo empty world file path
    gazebo_world = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'worlds',
        'empty.world'
    )

    return LaunchDescription([
        # Declare the world file argument
        DeclareLaunchArgument(
            'world',
            default_value=gazebo_world,
            description='Gazebo world file'
        ),

        # Start Gazebo server with factory plugin
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so', 
                 '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
            output='screen'
        ),

        # Start Gazebo client
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Joint state publisher (for RViz)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Spawn the robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_car', '-topic', 'robot_description', 
                      '-x', '0', '-y', '0', '-z', '0.2'],
            output='screen'
        ),

        # Optional: Start RViz2 with a basic config
        # Uncomment and modify path as needed
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/akash/ros2_ws/src/simple_car/rviz/simple_car.rviz']
        ),
    ])
    