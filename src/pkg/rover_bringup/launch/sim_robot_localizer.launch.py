#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from math import pi
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    namespace=LaunchConfiguration('namespace')

    

    declare_namespace=DeclareLaunchArgument('namespace',
                                            default_value='master',
                                            description='Namespace for slam node')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    
    ekf_params_file=LaunchConfiguration('ekf_params_file')
    declare_ekf_params_file_cmd=DeclareLaunchArgument('ekf_params_file',
                            default_value=os.path.join(get_package_share_directory
                                    ("rover_bringup"),
                                   'config', 'localization_ekf_master.yaml'))

    # Start robot localization using an Extended Kalman filter   
    localization_node = Node(
    	package='robot_localization',      
    	executable='ekf_node',
    	name='ekf_filter_node',
    	output='screen',
        namespace=namespace,
        
    	parameters=[ekf_params_file, {'use_sim_time': use_sim_time}]
    	)
    

   
    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_ekf_params_file_cmd)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(localization_node)
    

    
    return ld