from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.actions

def generate_launch_description():
    # Path to the URDF file and configurations
    pkg_wheelchair = get_package_share_directory('wheelchair')
    rviz_config_file = os.path.join(pkg_wheelchair, 'rviz', 'wheelchair.rviz')
    mapviz_config_file = os.path.join(pkg_wheelchair, "config", "gps_wpf_demo.mvc")

    urdf_file = "/home/rania/ros2_obstacle_ws/src/ros2_obstacle_avoidance/urdf/robot_real.urdf"
    
    # Read the URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    # Define launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Gazebo simulation launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': '-r ~/ros2_obstacle_ws/src/ros2_obstacle_avoidance/worlds/test.sdf'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    # Spawn the robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_file,
            '-name', 'wheelchair',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
            #'-Y', '-0.8'
        ],
        output='screen'
    )
    # Gazebo Bridge Node
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
        ],
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Define the launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo_launch,
        spawn_robot,
        robot_state_publisher,
        joint_state_publisher,
        gz_bridge_node,
        rviz_node,
        Node(
            package='ros2_obstacle_avoidance',
            executable='fuzzy_avoider',
            name='fuzzy_obstacle_avoider',
            output='screen',
            parameters=[{
                'safe_distance': 0.5,
                'stuck_threshold': 0.3,
                'linear_speed': 0.2,
                'angular_speed': 0.6
}]
        )
        #dqn_navigation_node,
        #navsat_transform_node,
    ])