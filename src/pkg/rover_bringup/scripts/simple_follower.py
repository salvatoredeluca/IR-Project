#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import math

class SimpleFollower(Node):
    def __init__(self):
        super().__init__('simple_follower')
        
        # Publisher per inviare il goal
        self.goal_pub = self.create_publisher(PoseStamped, 'slave/goal_pose', 10)
        
        # TF2 per leggere posizioni
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Timer per aggiornare il goal del slave
        self.timer = self.create_timer(0.8, self.follow_master)
        
        self.get_logger().info('Follower avviato')

    def follow_master(self):
        try:
            # Leggi posizione del master
            transform = self.tf_buffer.lookup_transform(
                'global', 'master/base_link', rclpy.time.Time())
            
            # Calcola posizione slave (2 metri dietro)
            master_x = transform.transform.translation.x
            master_y = transform.transform.translation.y
            
            slave_x = master_x - 0.5  # 2m dietro sull'asse X
            slave_y = master_y - 0.5
            
            # Crea goal per slave
            goal = PoseStamped()
            goal.header.frame_id = 'global'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = slave_x
            goal.pose.position.y = slave_y
            goal.pose.orientation.w = 1.0  # Orientamento neutro
            
            # Invia goal
            self.goal_pub.publish(goal)
            self.get_logger().info(f'Slave goal: ({slave_x:.1f}, {slave_y:.1f})')
            
        except Exception as e:
            self.get_logger().debug(f'Errore: {e}')

def main():
    rclpy.init()
    node = SimpleFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()