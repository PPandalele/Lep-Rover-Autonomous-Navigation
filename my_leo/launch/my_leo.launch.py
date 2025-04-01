import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

import xacro

def generate_launch_description():
    ld = LaunchDescription()

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'my_leo'

    # Set ignition resource path (so it can find your world files)
    ign_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',
    value=[os.path.join(get_package_share_directory(pkg_name),'worlds')])

    # Include the Gazebo launch file
    launch_gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
    launch_arguments={
    'gz_args' : '-r empty.sdf'
    }.items(),
    )

    # Add features
    gz_spawn_objects = Node(package='ros_gz_sim', executable='create',
    arguments=['-file', os.path.join(get_package_share_directory(pkg_name), 'worlds', 'tb3.sdf'),
    '-x', '0.0',
    '-y', '0.5',
    '-z', '0.0'],
    output='screen'
    )

    urdf_file = os.path.join(get_package_share_directory('my_leo'), 'urdf', 'leo_sim.urdf.xacro')
    robot_desc = xacro.process_file(urdf_file).toxml()

    # Launch robot state publisher node
    robot_state_publisher = Node(
        # namespace="leo_rover",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # name="robot_state_publisher",
        output='screen',
        parameters=[
            {"robot_description": robot_desc},
        ],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )
    ld.add_action(joint_state_publisher)

    # Spawn a robot inside a simulation
    leo_rover = Node(
        # namespace="leo_rover",
        package="ros_gz_sim",
        executable="create",
        # name="ros_gz_sim_create",
        # output="both",
        output='screen',
        arguments=[
            "-topic",
            "/robot_description",
            # "-name",
            # "leo_rover",
            "-z",
            "0.5",
        ],
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            "/model/leo_rover/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/model/leo_rover/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/model/leo_rover/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/model/leo_rover/imu/data_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/model/leo_rover/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
            '/model/leo_rover/scan'+'@sensor_msgs/msg/LaserScan'+'['+'ignition.msgs.LaserScan',
        ],
        parameters=[
            {
                "qos_overrides ./model/leo_rover.subscriber.reliability": 'reliable',
            }
        ],
        remappings= [
            ('/model/leo_rover/cmd_vel',  '/cmd_vel'),
            ('/model/leo_rover/odom',  '/wheel_odom_with_covariance'),
            ('/model/leo_rover/tf',  '/tf'),
            ('/model/leo_rover/imu/data_raw',  '/imu/data_raw'),
            ('/model/leo_rover/joint_states',  '/joint_states'),
            ('/model/leo_rover/scan',  '/scan'),
            ],
        output="screen",
    )

    # Camera image bridge
    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge",
        arguments=["/model/leo_rover/camera/image_raw"],
        remappings= [('/model/leo_rover/camera/image_raw',  '/camera/image_raw'),],
        output="screen",
    )

    # Rviz node
    node_rviz = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory(pkg_name), '/launch', '/rviz_leo.launch.py']),
    launch_arguments={}.items(),
    )

    navigation = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory(pkg_name), '/launch', '/navigation.launch.py']),
    launch_arguments={}.items(),
    )

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('rplidar_ros'), '/launch', '/rplidar_a2m12_launch.py']),
        launch_arguments={}.items(),
    )

    ekf_localization = Node(
       package="robot_localization",
       executable="ekf_node",
       name="ekf_filter_node",
       output = 'screen',
       parameters=[os.path.join(get_package_share_directory(pkg_name), 'config', 'ekf.yaml')],
    )

    joy_stick = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory(pkg_name), '/launch', '/teleop_joy.launch.py']),
    launch_arguments={}.items(),
    )



    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=False))
    #ld.add_action(ign_resource_path)
    #ld.add_action(launch_gazebo)
    #ld.add_action(gz_spawn_objects)
    ld.add_action(robot_state_publisher)
    ld.add_action(leo_rover)
    #ld.add_action(topic_bridge)
    #ld.add_action(image_bridge)
    # ld.add_action(ekf_localization)
    ld.add_action(node_rviz)    
    ld.add_action(navigation)
    # ld.add_action(joy_stick)
    # ld.add_action(launch_slamtoolbox)
    ld.add_action(rplidar)
    
    return ld
