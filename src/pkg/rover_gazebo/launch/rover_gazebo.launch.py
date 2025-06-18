import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction


def generate_launch_description():
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = os.path.join(get_package_share_directory(
        'rover_description_pkg'), 'urdf', 'rover.xacro')
    
    urdf_slave = os.path.join(get_package_share_directory(
        'rover_description_pkg'), 'urdf', 'rover_slave.xacro')


    world = LaunchConfiguration('world')
    frame_prefix = LaunchConfiguration("frame_prefix")
    tf_prefix = LaunchConfiguration("tf_prefix")   

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "frame_prefix",
            default_value='rover/',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    
    robot_desc_master = ParameterValue(Command(
            ['xacro ', urdf,
             " ",            
            "tf_prefix:=master_",  
            tf_prefix,
            " ",
            "frame_prefix:=master/",  
            frame_prefix]),value_type=str)
    


    robot_desc_slave = ParameterValue(Command(
            ['xacro ', urdf_slave,
             " ",            
            "tf_prefix:=slave_",  
            tf_prefix,
            " ",
            "frame_prefix:=slave/",  
            frame_prefix]),value_type=str)
    

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='maze.sdf',
        description='World file to use in Gazebo')
    
    gz_world_arg = PathJoinSubstitution([
        get_package_share_directory('rover_gazebo'), 'worlds', world])

    # Include the gz sim launch file  
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : gz_world_arg 
        }.items()
    )
    
    # Spawn Rover Robot Master
    gz_spawn_entity_master = Node(
        package="ros_gz_sim",
        executable="create",
        
        arguments=[
            "-topic", "master/robot_description",
            "-name", "prisma_rover_master",
            "-allow_renaming", "true",
            "-z", "0.1","-x","0.4","-y","0.1"
        ]
    )
    
    
    # Spawn Rover Robot Slave
    gz_spawn_entity_slave = Node(
        package="ros_gz_sim",
        executable="create",
        
        arguments=[
            "-topic", "slave/robot_description",
            "-name", "prisma_rover_slave",
            "-allow_renaming", "true",
            "-z", "0.7","-x","1","-y","0.2"
        ]
    )
    
    
    
    
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "slave/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
             "master/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/odom/wheels@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",          # @ is for a 1 to 1 conversion, [ is for a 1 to multiples conversion
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/color/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/depth/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/depth/color/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
        ],
    )

    # Robot state publisher
    params = {'use_sim_time': use_sim_time, 'robot_description': robot_desc_master, "tf_prefix":tf_prefix,"frame_prefix":frame_prefix}
    start_robot_state_publisher_cmd_master = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_master',
            namespace='master',
            output='both',
            parameters=[params],
            arguments=[])
    
    params = {'use_sim_time': use_sim_time, 'robot_description': robot_desc_slave, "tf_prefix":tf_prefix,"frame_prefix":frame_prefix}
    start_robot_state_publisher_cmd_slave = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_slave',
            namespace='slave',
            output='both',
            parameters=[params],
            arguments=[])

    delayed_spawn_slave = TimerAction(
    period=3.5,
    actions=[gz_spawn_entity_slave]
    )

    # Create the launch description and populate
    ld = LaunchDescription(declared_arguments)

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)

    # Launch Gazebo
    ld.add_action(gz_sim)
    ld.add_action(gz_spawn_entity_master)
    ld.add_action(delayed_spawn_slave)
    ld.add_action(gz_ros2_bridge)

    # Launch Robot State Publisher
    ld.add_action(start_robot_state_publisher_cmd_master)
    ld.add_action(start_robot_state_publisher_cmd_slave)
    return ld
