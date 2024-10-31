#!/usr/bin/env python3
import sys
#import rospy

import rclpy 
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg  import PoseStamped, TransformStamped, Quaternion, Pose
import numpy as np
import time
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import math

# from tf.transformations import quaternion_from_euler

"""
The class of the pid controller.
"""
class AprilToWorldCoords(Node):

    def __init__(self):
        super().__init__('april_to_world_coords')
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(Pose, '/robot_coords', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1, self.check_transform)
   

    def check_transform(self):
        
        #April Tag 0
        marker_0_world_pos = TransformStamped()
        marker_0_world_pos.header.stamp = self.get_clock().now().to_msg()
        marker_0_world_pos.header.frame_id = 'world'
        marker_0_world_pos.child_frame_id = 'marker_0'
        marker_0_world_pos.transform.translation.x = 0.8
        marker_0_world_pos.transform.translation.y = 0.0
        marker_0_world_pos.transform.translation.z = 0.0
        marker_0_world_pos.transform.rotation.x = -0.5
        marker_0_world_pos.transform.rotation.y = -0.5
        marker_0_world_pos.transform.rotation.z = 0.5
        marker_0_world_pos.transform.rotation.w = 0.5

        self.tf_broadcaster.sendTransform(marker_0_world_pos)

        #April Tag 1
        marker_1_world_pos = TransformStamped()
        marker_1_world_pos.header.stamp = self.get_clock().now().to_msg()
        marker_1_world_pos.header.frame_id = 'world'
        marker_1_world_pos.child_frame_id = 'marker_1'
        marker_1_world_pos.transform.translation.x = 0.5
        marker_1_world_pos.transform.translation.y = 1.3
        marker_1_world_pos.transform.translation.z = 0.0
        marker_1_world_pos.transform.rotation.x =  4.3297802811774664e-17
        marker_1_world_pos.transform.rotation.y =  -0.7071067811865475
        marker_1_world_pos.transform.rotation.z = 0.7071067811865476
        marker_1_world_pos.transform.rotation.w = 4.329780281177467e-17
        self.tf_broadcaster.sendTransform(marker_1_world_pos)

        # #April Tag 2
        marker_2_world_pos = TransformStamped()
        marker_2_world_pos.header.stamp = self.get_clock().now().to_msg()
        marker_2_world_pos.header.frame_id = 'world'
        marker_2_world_pos.child_frame_id = 'marker_2'
        marker_2_world_pos.transform.translation.x = 0.0
        marker_2_world_pos.transform.translation.y = 1.0
        marker_2_world_pos.transform.translation.z = 0.0
        marker_2_world_pos.transform.rotation.x = -0.5
        marker_2_world_pos.transform.rotation.y = 0.5
        marker_2_world_pos.transform.rotation.z = -0.5
        marker_2_world_pos.transform.rotation.w = 0.5

        self.tf_broadcaster.sendTransform(marker_2_world_pos)


        #April Tag 3
        marker_3_world_pos = TransformStamped()
        marker_3_world_pos.header.stamp = self.get_clock().now().to_msg()
        marker_3_world_pos.header.frame_id = 'world'
        marker_3_world_pos.child_frame_id = 'marker_3'
        marker_3_world_pos.transform.translation.x = -0.2
        marker_3_world_pos.transform.translation.y = -0.1
        marker_3_world_pos.transform.translation.z = 0.0
        marker_3_world_pos.transform.rotation.x = 0.21263110997159382
        marker_3_world_pos.transform.rotation.y = 0.6743797232066279
        marker_3_world_pos.transform.rotation.z = 0.6743797232066278
        marker_3_world_pos.transform.rotation.w = 0.21263110997159385

        self.tf_broadcaster.sendTransform(marker_3_world_pos)


        try:
            transform = self.tf_buffer.lookup_transform('world', 'camera', rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f"{str(e)}")
            return

        object1_pose = Pose()
        object1_pose.position.x = transform.transform.translation.x
        object1_pose.position.y = transform.transform.translation.y
        object1_pose.position.z = transform.transform.translation.z
        object1_pose.orientation = transform.transform.rotation 

    

        self.publisher.publish(object1_pose)
        self.get_logger().info(f"x {object1_pose.position.x} y {object1_pose.position.y} z {object1_pose.position.z}")
                
def main():
    rclpy.init()

    april_to_world = AprilToWorldCoords()
    try:
        rclpy.spin(april_to_world)
    except KeyboardInterrupt as k:
        print('exiting :)')

    april_to_world.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

