from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_maf = get_package_share_directory('multiagent_formations')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gz_model_path = PathJoinSubstitution([pkg_ros_gz_sim, 'models'])
    # gz_worlds_path = PathJoinSubstitution([pkg_maf, 'worlds'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            choices=['empty'],
            description='World to load into Gazebo'
        ),
        SetLaunchConfiguration(name='world_file', 
                            #    value=[gz_worlds_path,TextSubstitution(text='/'),LaunchConfiguration('world'), 
                            #           TextSubstitution(text='.sdf')]),
                               value=[LaunchConfiguration('world'), 
                                      TextSubstitution(text='.sdf')]),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [PathJoinSubstitution([LaunchConfiguration('world_file')])],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/model/vehicle_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/model/vehicle_1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/model/vehicle_1/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/model/vehicle_2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/model/vehicle_2/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/model/vehicle_2/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/model/vehicle_3/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/model/vehicle_3/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/model/vehicle_3/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/model/vehicle_4/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/model/vehicle_4/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/model/vehicle_4/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            parameters=[{'qos_overrides./model/vehicle_1.subscriber.reliability': 'reliable'},
                        {'qos_overrides./model/vehicle_2.subscriber.reliability': 'reliable'},
                        {'qos_overrides./model/vehicle_3.subscriber.reliability': 'reliable'},
                        {'qos_overrides./model/vehicle_4.subscriber.reliability': 'reliable'}],
            remappings=[('/model/vehicle_1/pose', '/tf'),
                        ('/model/vehicle_2/pose', '/tf'),
                        ('/model/vehicle_3/pose', '/tf'),
                        ('/model/vehicle_4/pose', '/tf')],
            output='screen'
        ),
        Node(
            package='multiagent_formations',
            executable='formation_control_node.py',
            name='formation_control_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])