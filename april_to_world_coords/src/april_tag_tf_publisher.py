#!/usr/bin/env python3
import sys
#import rospy

import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
from math import radians

class AprilTagPublisher(Node):    
    def __init__(self):
        super().__init__('static_transform_publisher')
        self.br = tf2_ros.StaticTransformBroadcaster(self)

        # name, x, y, z, xrot, yrot, zrot, wrot
        frame0_tf = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        frame1_tf = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        frame2_tf = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        frame3_tf = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        self.send_tf('marker_0', frame0_tf)
        #self.send_tf('marker_1', frame1_tf)
        #self.send_tf('marker_2', frame2_tf)
        #self.send_tf('marker_3', frame3_tf)

    def send_tf(self, name, pos):
        # Create a static transform
        static_transform = geometry_msgs.msg.TransformStamped()

        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'world_origin'      # Parent frame
        static_transform.child_frame_id = name                 # Child frame

        # Set the translation (x, y, z)
        static_transform.transform.translation.x = pos[0]
        static_transform.transform.translation.y = pos[1]
        static_transform.transform.translation.z = pos[2]

        # Set the rotation (as a quaternion)
        static_transform.transform.rotation.x = pos[3]
        static_transform.transform.rotation.y = pos[4]
        static_transform.transform.rotation.z = pos[5]
        static_transform.transform.rotation.w = pos[6]

        # Publish the static transform
        self.br.sendTransform(static_transform)

if __name__ == '__main__':
    rclpy.init()
    
    april_tag_publisher = AprilTagPublisher()

    rclpy.spin(april_tag_publisher)

    april_tag_publisher.destroy_node()
    rclpy.shutdown()