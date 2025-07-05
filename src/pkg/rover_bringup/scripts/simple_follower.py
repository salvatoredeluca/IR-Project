#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import math

class SmartFollower(Node):
    def __init__(self):
        super().__init__('smart_follower')
        
        self.goal_pub = self.create_publisher(PoseStamped, 'slave/goal_pose', 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.trail = []
        self.current_index = 0
        
        self.min_dist = 1.5            # distanza minima per registrare un punto nella trail
        self.reach_threshold = 0.9     # distanza per considerare raggiunto un goal
        self.follow_threshold = 1.7    # distanza entro cui si segue direttamente il master
        self.stop_threshold = 1.3      # distanza minima dal master sotto cui NON si manda più goal
        
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.get_logger().info('SmartFollower avviato')

    def timer_callback(self):
        try:
            # === Posizione MASTER ===
            tf_master = self.tf_buffer.lookup_transform(
                'global', 'master/base_link', rclpy.time.Time())
            
            master_pose = PoseStamped()
            master_pose.header.frame_id = 'global'
            master_pose.header.stamp = self.get_clock().now().to_msg()
            master_pose.pose.position.x = tf_master.transform.translation.x
            master_pose.pose.position.y = tf_master.transform.translation.y
            master_pose.pose.position.z = tf_master.transform.translation.z
            master_pose.pose.orientation = tf_master.transform.rotation

            # === Posizione SLAVE ===
            tf_slave = self.tf_buffer.lookup_transform(
                'global', 'slave/base_link', rclpy.time.Time())
            slave_x = tf_slave.transform.translation.x
            slave_y = tf_slave.transform.translation.y

            dx = master_pose.pose.position.x - slave_x
            dy = master_pose.pose.position.y - slave_y
            distance_to_master = math.sqrt(dx*dx + dy*dy)

            if distance_to_master <= self.follow_threshold:
               
                self.trail = []
                self.current_index = 0

                if distance_to_master <= self.stop_threshold:
                   
                    self_pose = PoseStamped()
                    self_pose.header.frame_id = 'global'
                    self_pose.header.stamp = self.get_clock().now().to_msg()
                    self_pose.pose.position.x = slave_x
                    self_pose.pose.position.y = slave_y
                    self_pose.pose.orientation = tf_slave.transform.rotation

                    self.goal_pub.publish(self_pose)
                    self.get_logger().info(f'Slave è troppo vicino al master ({distance_to_master:.2f} m), si ancora in posizione')
                    return
                else:
                   
                    self.goal_pub.publish(master_pose)
                    self.get_logger().info(f'Seguo direttamente il master ({distance_to_master:.2f} m)')
                    return

            
            if len(self.trail) == 0:
                self.trail.append(master_pose)
            else:
                last = self.trail[-1]
                dx = master_pose.pose.position.x - last.pose.position.x
                dy = master_pose.pose.position.y - last.pose.position.y
                dist = math.sqrt(dx*dx + dy*dy)
                if dist >= self.min_dist:
                    self.trail.append(master_pose)
                    self.get_logger().info(f'Aggiunto punto alla trail ({master_pose.pose.position.x:.2f}, {master_pose.pose.position.y:.2f})')

            
            if len(self.trail) == 0:
                self.get_logger().warn('Trail vuota, non posso seguire nulla')
                return

            current_goal = self.trail[self.current_index]
            dxg = current_goal.pose.position.x - slave_x
            dyg = current_goal.pose.position.y - slave_y
            dist_to_goal = math.sqrt(dxg*dxg + dyg*dyg)

            if dist_to_goal < self.reach_threshold and self.current_index < len(self.trail) - 1:
                self.current_index += 1
                current_goal = self.trail[self.current_index]
                self.get_logger().info(f'Raggiunto goal, passo al successivo [{self.current_index}]')

            self.goal_pub.publish(current_goal)
            self.get_logger().info(f'Seguo la trail: goal {self.current_index}/{len(self.trail)} a distanza {dist_to_goal:.2f} m')

        except Exception as e:
            self.get_logger().debug(f'Errore nel timer_callback: {e}')



    


def main():
    rclpy.init()
    node = SmartFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
