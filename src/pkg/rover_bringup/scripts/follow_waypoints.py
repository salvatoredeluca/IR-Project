#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped


def main():
    rclpy.init()
    
    # Inizializza navigator con namespace
    navigator = BasicNavigator(namespace='master')
    
    # Aspetta che Nav2 sia pronto (senza AMCL)
    # navigator.waitUntilNav2Active(localizer='')
    print("Nav2 è pronto!")
    
    # Crea goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'global'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.66
    goal_pose.pose.position.y = -3.0
    goal_pose.pose.orientation.w = 0.0
    
    # Invia goal
    navigator.goToPose(goal_pose)
    print("Goal inviato!")
    
    # Aspetta completamento
    while not navigator.isTaskComplete():
        pass
    
    # Controlla risultato
    result = navigator.getResult()
    if result == 3:  # SUCCEEDED
        print("Goal raggiunto!")
    else:
        print(f"Goal fallito: {result}")
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()