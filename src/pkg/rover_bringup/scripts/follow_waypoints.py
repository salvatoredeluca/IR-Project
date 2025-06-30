#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler
from ament_index_python.packages import get_package_share_directory
import os
import yaml

waypoints_yaml_path=os.path.join(get_package_share_directory('rover_bringup'), "config", "waypoints.yaml")
with open(waypoints_yaml_path, 'r') as yaml_file:
        yaml_content = yaml.safe_load(yaml_file)

def main():
    rclpy.init()
    
    # Inizializza navigator con namespace
    navigator = BasicNavigator(namespace='master')
    
    # Aspetta che Nav2 sia pronto (senza AMCL)
    # navigator.waitUntilNav2Active(localizer='')
    print("Nav2 è pronto!")
    

    def create_pose(transform):
        name= transform["name"]
        pose = PoseStamped()
        pose.header.frame_id = 'global'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = transform["position"]["x"]
        pose.pose.position.y = transform["position"]["y"]
        pose.pose.position.z = transform["position"]["z"]
        roll = transform["orientation"]["roll"]
        pitch = transform["orientation"]["pitch"]
        yaw = transform["orientation"]["yaw"]
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose, name

    goals = list(map(create_pose, yaml_content["waypoints"]))
    goal_poses = []

    for pose,name in goals:
        goal_poses.append(pose)
    
    
    # Invia goal
    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i=0
    while not navigator.isTaskComplete():
    
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    exit(0)

  

if __name__ == '__main__':
    main()