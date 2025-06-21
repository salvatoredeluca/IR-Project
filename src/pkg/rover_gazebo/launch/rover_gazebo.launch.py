import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = os.path.join(get_package_share_directory(
        'rover_description_pkg'), 'urdf', 'rover.xacro')
    
    x=LaunchConfiguration('x')

    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )



    declared_arguments.append(
        DeclareLaunchArgument(
            "x",
            default_value='',
        )
    )



    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='/',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
  
    prefix = LaunchConfiguration("prefix")
    namespace = LaunchConfiguration('namespace')  

    robot_description_content = ParameterValue(Command(
            ['xacro ', urdf,
             " ",            
            "prefix:=",  
            prefix,' ',
            'namespace:=',
            namespace
            ]),value_type=str)
    

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    
    # Spawn Rover Robot
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=namespace,
        arguments=[
            "-topic",  "robot_description",
            "-name", [prefix],
            "-allow_renaming", "true",
            "-z", "0.1",
            "-x",x,
            "-entity",[prefix],
            
        ]
    )
    
    # gz_ros2_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
    #         "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
    #         "/odom/wheels@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
    #         "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",          # @ is for a 1 to 1 conversion, [ is for a 1 to multiples conversion
    #         '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
    #         '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
    #         '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
    #         '/color/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
    #         '/depth/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
    #         '/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
    #         '/depth/color/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
    #     ],
    # )

    


    


    # Robot state publisher
    params = {'use_sim_time': use_sim_time, 'robot_description': robot_description_content}
    start_robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='both',
            parameters=[params],
            arguments=[])

    # Create the launch description and populate
    ld = LaunchDescription(declared_arguments)

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    
    ld.add_action(gz_spawn_entity)
    
    # ld.add_action(gz_ros2_bridge)
    # Launch Robot State Publisher
    ld.add_action(start_robot_state_publisher_cmd)

    return ld