#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler
from ament_index_python.packages import get_package_share_directory
import os
import yaml

class WaypointNavigatorNode(Node):
    def __init__(self):
        super().__init__('waypoint_navigator_node')

        self.get_logger().info('Inizializzo il navigatore Nav2...')

        # Inizializza il BasicNavigator
        self.navigator = BasicNavigator(namespace='master')

        # Carica i waypoints dal file YAML
        waypoints_yaml_path = os.path.join(
            get_package_share_directory('rover_bringup'),
            "config",
            "waypoints.yaml"
        )
        with open(waypoints_yaml_path, 'r') as yaml_file:
            yaml_content = yaml.safe_load(yaml_file)

        # Prepara le pose
        self.goal_poses = []
        for transform in yaml_content["waypoints"]:
            pose = PoseStamped()
            pose.header.frame_id = 'global'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.position.x = transform["position"]["x"]
            pose.pose.position.y = transform["position"]["y"]
            pose.pose.position.z = transform["position"]["z"]
            roll = transform["orientation"]["roll"]
            pitch = transform["orientation"]["pitch"]
            yaw = transform["orientation"]["yaw"]
            q = quaternion_from_euler(roll, pitch, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            self.goal_poses.append(pose)

        self.get_logger().info(f"Trovati {len(self.goal_poses)} waypoints, invio alla navigazione.")

        # Avvia la navigazione
        self.nav_start = self.navigator.get_clock().now()
        self.navigator.followWaypoints(self.goal_poses)

        # Crea un timer per monitorare lo stato periodicamente
        self.timer = self.create_timer(1.0, self.navigation_feedback_cb)

        self.feedback_counter = 0

    def navigation_feedback_cb(self):
        # Se task è completo, fai shutdown
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                self.get_logger().warn('Goal was canceled!')
            elif result == TaskResult.FAILED:
                self.get_logger().error('Goal failed!')
            else:
                self.get_logger().error('Goal has invalid return status!')
            rclpy.shutdown()
            return

        # Altrimenti, stampa il feedback ogni 5 chiamate
        self.feedback_counter += 1
        feedback = self.navigator.getFeedback()
        if feedback and self.feedback_counter % 5 == 0:
            self.get_logger().info(
                f"Executing waypoint {feedback.current_waypoint + 1}/{len(self.goal_poses)}"
            )

        # Timeout per demo (esempio: 10 minuti)
        now = self.navigator.get_clock().now()
        if now - self.nav_start > Duration(seconds=600):
            self.get_logger().warn('Navigation timed out. Cancelling task...')
            self.navigator.cancelTask()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigatorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
