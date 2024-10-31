#!/usr/bin/env python3
import sys
#import rospy

import rclpy 
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from mpi_control import MegaPiController
import numpy as np
import time

# 0:(0,0,0). 1:(1,0,0). 2:(1,2,pi). 3:(0,0,0)
WP1_STRAIGHT = 0
WP2_TURN = 1
WP2_STRAIGHT = 2
WP2_TURN2 = 3
WP3_TURN = 4
WP3_STRAIGHT = 5
WP3_TURN2 = 6
DONE = 7

"""
The class of the pid controller.
"""
class PIDcontroller(Node):
    def __init__(self):
        super().__init__('PID_Controller_NodePub')
        self.target = None
        self.timestep = 0.1
        self.maximumValue = 0.1
        
        self.mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=True)
        self.r = 0.025 # radius of the wheel
        self.lx = 0.055 # half of the distance between front wheel and back wheel
        self.ly = 0.07 # half of the distance between left wheel and right wheel
        self.calibration = -10.0
        self.jacobian_matrix = np.array([[1, -1, -(self.lx + self.ly)],
                                     [1, 1, (self.lx + self.ly)],
                                     [1, 1, -(self.lx + self.ly)],
                                     [1, -1, (self.lx + self.ly)]]) / self.r

        self.subscriber = self.create_subscription(PoseStamped, '/april_poses', self.update, 10)
        self.state = None

    def setTargetMarker(self, target):
        self.target = target

    def update(self, msg):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        if msg.header.frame_id != self.target:
            return
        
        #pid.publisher_.publish(genTwistMsg([0.1, 0.0, msg.pose.position.x]))
        #pid.publisher_.publish(genTwistMsg([10.0, 0.0, 0.0]))

        if self.state == WP1_STRAIGHT:
            if msg.pose.position.z < 0.3:
                pid.mpi_ctrl.carStop()
                self.setTargetMarker('1')
                self.state = WP2_TURN
                time.sleep(1)
                print('WP2_TURN')
                self.setMotorsTurn(msg.pose.position.x)
            else:
                self.setMotorsStraight(msg.pose.position.x)
        elif self.state == WP2_TURN:
            if msg.pose.position.y > -1:
                pid.mpi_ctrl.carStop()
                self.state = WP2_STRAIGHT
                time.sleep(1)
                print('WP2_STRAIGHT')
            else:
                self.setMotorsTurn(msg.pose.position.x)
        elif self.state == WP2_STRAIGHT:
            if msg.pose.position.z < 0.3:
                pid.mpi_ctrl.carStop()
                self.setTargetMarker('2')
                self.state = WP2_TURN2
                time.sleep(1)
                print('WP2_TURN2')
                self.setMotorsTurn(msg.pose.position.x)
            else:
                self.setMotorsStraight(msg.pose.position.x)
        elif self.state == WP2_TURN2:
            if msg.pose.position.y > -1:
                pid.mpi_ctrl.carStop()
                self.setTargetMarker('3')
                self.state = WP3_TURN
                time.sleep(1)
                print('WP3_TURN')
                self.setMotorsTurn(msg.pose.position.x)
            else:
                self.setMotorsTurn(msg.pose.position.x)
        elif self.state == WP3_TURN:
            if msg.pose.position.y > -0.5:
                pid.mpi_ctrl.carStop()
                self.state = WP3_STRAIGHT
                time.sleep(1)
                print('WP3_STRAIGHT')
            else:
                self.setMotorsTurn(msg.pose.position.x)
        elif self.state == WP3_STRAIGHT:
            if msg.pose.position.z < 0.5:
                pid.mpi_ctrl.carStop()
                self.setTargetMarker('0')
                self.state = WP3_TURN2
                time.sleep(1)
                print('WP3_TURN2')
                self.setMotorsTurn(msg.pose.position.x)
            else:
                self.setMotorsStraight(msg.pose.position.x)
        elif self.state == WP3_TURN2:
            if msg.pose.position.y > -1:
                pid.mpi_ctrl.carStop()
                self.state = DONE
                time.sleep(1)
                print('DONE')
            else:
                self.setMotorsTurn(msg.pose.position.x)
        elif self.state == DONE:
            pid.mpi_ctrl.carStop()
            pid.mpi_ctrl.close()
            raise Exception
        

    def setMotorsStraight(self, xpos):
        desired_twist = self.calibration * np.array([[0.1], [0.0], [xpos*0.75]]) # CHANGE TO CALIBRATE
        # calculate the desired wheel velocity
        result = np.dot(self.jacobian_matrix, desired_twist)
        # send command to each wheel
        self.mpi_ctrl.setFourMotors(result[0][0], result[1][0], result[2][0], result[3][0])

    def setMotorsTurn(self, xpos):
        desired_twist = np.array([[0.0], [0.0], [8-xpos]]) # CHANGE TO CALIBRATE
        # calculate the desired wheel velocity
        result = np.dot(self.jacobian_matrix, desired_twist)
        # send command to each wheel
        self.mpi_ctrl.setFourMotors(result[0][0], result[1][0], result[2][0], result[3][0])


def genTwistMsg(desired_twist):
    """
    Convert the twist to twist msg.
    """
    twist_msg = Twist()
    twist_msg.linear.x = desired_twist[0] 
    twist_msg.linear.y = desired_twist[1] 
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = desired_twist[2]
    return twist_msg

def coord(twist, current_state):
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)
    


if __name__ == "__main__":
    rclpy.init()
    #rospy.init_node("hw1")
    #pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    
    pid = PIDcontroller()
    pid.setTargetMarker('0')
    pid.state = WP1_STRAIGHT

    try:
        rclpy.spin(pid)
    except Exception as e:
        pid.mpi_ctrl.carStop()
        time.sleep(1)