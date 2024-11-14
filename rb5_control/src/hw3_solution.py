#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseArray
import numpy as np
import math
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from rclpy.time import Duration
import math
from math import copysign, fabs, sqrt, pi, sin, cos, asin, acos, atan2, exp, log

class KalmanFilter(object):

    
    def predict(self, u = 0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    # sensor_measurements = [[tag_id, x,y], [tag_id,x,y]]
    def update(self, sensor_measurements):

        if len(self.x) == 3 or sensor_measurements == []: # Skip update if no april tags are currently visible
            return self.x

        visible_tag_count = len(sensor_measurements)
        
        self.R = np.diag(np.full(visible_tag_count * 2, 0.6))         
        self.H = np.zeros((visible_tag_count * 2, len(self.x)))
        z = np.array([0.0] * visible_tag_count * 2) 
        
        theta_r = self.x[2]
        # print(" SENSOR: {}".format(sensor_measurements))
        for idx in range(len(sensor_measurements)):
            tag_id = sensor_measurements[idx][0]
            april_robot_pos_x = sensor_measurements[idx][1] 
            april_robot_pos_y = sensor_measurements[idx][2]
            # print(" {} - {} ".format(april_robot_pos_x, april_robot_pos_y))
            z[idx*2] = april_robot_pos_x
            z[idx*2 + 1] = april_robot_pos_y

            self.H[idx*2][0] = -np.cos(theta_r)
            self.H[idx*2][1] = -np.sin(theta_r)
            self.H[idx*2][self.april_tags[tag_id]] = np.cos(theta_r)
            self.H[idx*2][self.april_tags[tag_id] + 1] = np.sin(theta_r)



            self.H[idx*2+1][0] = np.sin(theta_r)
            self.H[idx*2+1][1] = -np.cos(theta_r)
            self.H[idx*2+1][self.april_tags[tag_id]] = -np.sin(theta_r)
            self.H[idx*2+1][self.april_tags[tag_id] + 1] = np.cos(theta_r)


        z0 = np.dot(self.H, self.x)
        # print(" H: {}".format(np.round(self.H,2)))  
        # print(" z: {} z0: {}".format(z, np.round(z0,2)))
        y = z - z0
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(len(self.x))
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), 
        	(I - np.dot(K, self.H)).T)  
        

        rounded_arr = np.round(self.x, 2)
        print("new state: {} ".format(rounded_arr))

        return self.x
    
        
    def robot_to_world_transform(self):
        """ Robot to world transformation matrix (inverse of world_to_robot_transform) """
        robot_world_pos = self.x[:3]  # Assuming robot's position and orientation are in x[:3]
        theta = robot_world_pos[2]  # robot's orientation (theta) in radians
      
        return np.array([
            [np.cos(theta), -np.sin(theta), robot_world_pos[0]], # x_r + x_a*cos(theta_r) - y_a*sin(theta_r) 
            [np.sin(theta), np.cos(theta), robot_world_pos[1]], # y_r + x_a*sin(theta_r) + y_a*cos(theta_r) 
            [0, 0, 1]
        ])


    def __init__(self, F = None, B = None, H = None, Q = None, R = None, P = None, x0 = None):
        self.n = 3
        dt = 1
        self.F = np.eye(self.n)
        self.H = np.array([1, 0, 0]).reshape(1, 3)
        self.B = np.array([[3 * dt, 0, 0], [0, 3 * dt, 0], [0, 0, 3 * dt]])
        self.Q = np.diag([.1, .1, .1]) if Q is None else Q
        self.R = None
        self.P = np.diag(np.full(self.n, 20)) if P is None else P
        self.x = np.array([0,0,0]) if x0 is None else x0
        self.april_tags = {}




    def new_april_tag(self, id, april_robot_pos):
        april_world_pos = np.dot(self.robot_to_world_transform(), april_robot_pos)
        self.april_tags[id] = len(self.x)
        # print(" NEW APRIL TAG POS_WORLD: {} POS_ROBOT: {} ".format(april_world_pos, april_robot_pos))

        self.x = np.append(self.x, april_world_pos[:2])
        # new_Q = np.diag(np.full(len(self.x), 0))
        # new_Q[:3, :3] = self.Q[:3, :3]
        # self.Q = new_Q
        self.Q = np.diag(np.full(len(self.x), 0.1))


        new_P = np.diag(np.full(len(self.x), 20))
        new_P[:3, :3] = self.P[:3, :3]
        self.P = new_P

        # self.R = np.diag(np.full(len(self.x) - 3, 0.05))         
        self.F = np.eye(len(self.x))

        new_B = np.zeros((len(self.x),3))
        new_B[:3, :3] = self.B[:3, :3]
        self.B = new_B

                
        print(self.x)


class RobotStateEstimator(Node):
    def __init__(self):
        super().__init__('robot_state_estimator')
        self.subscription = self.create_subscription(
            PoseArray,
            '/april_poses',
            self.april_pose_callback,
            10)


        # self.br = TransformBroadcaster(self)
        self.current_state = np.array([0.0, 0.0, 0.0])
        self.kf = KalmanFilter(x0 = self.current_state)
        self.sensor_measurements = [0] * (len(self.kf.x) - 3)
        self.april_updated = False
        self.april_tags = {}


    def update_state(self, motor_input):
        
        self.kf.predict(motor_input)
        if self.april_updated:
            self.current_state = self.kf.update(self.sensor_measurements)
            self.april_updated = False

        self.current_state = self.kf.x
        return self.current_state


    def april_pose_callback(self, msg):
        if len(msg.poses) < 1:
            self.sensor_measurements = []
            return

        pose_ids = msg.header.frame_id.split(',')[:-1]

        for april_tag in range(len(pose_ids)):
            tag_id = pose_ids[april_tag]
            pose_camera_apriltag = msg.poses[april_tag]


            if tag_id not in self.april_tags:
                self.april_tags[tag_id] = len(self.kf.x)
                # if last argument of this line is not 1, it messes stuff up :P
                self.kf.new_april_tag(tag_id, np.array([pose_camera_apriltag.position.z, -pose_camera_apriltag.position.x, 1]))


        april_robot_poses = []

        for april_tag in range(len(pose_ids)):
            
            
            tag_id = pose_ids[april_tag]
            pose_camera_apriltag = msg.poses[april_tag]            

            april_robot_pos_x = pose_camera_apriltag.position.z
            april_robot_pos_y = -pose_camera_apriltag.position.x
            april_robot_pose = [tag_id, april_robot_pos_x, april_robot_pos_y]
            april_robot_poses.append(april_robot_pose)

        
        self.sensor_measurements = april_robot_poses
        self.april_updated = True



class PIDcontroller(Node):
    def __init__(self, Kp, Ki, Kd):
        super().__init__('PID_Controller_NodePub')
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.02
        self.publisher_ = self.create_publisher(Twist, '/twist', 10)

    def setTarget(self, target):
        """
        Set the target pose.
        """
        self.I = np.array([0.0, 0.0, 0.0])
        self.lastError = np.array([0.0, 0.0, 0.0])
        self.target = np.array(target)

    def getError(self, currentState, targetState):
        """
        Return the difference between two states.
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result

    def setMaximumUpdate(self, mv):
        """
        Set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState):
        """
        Calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)
        P = self.Kp * e
        self.I += self.Ki * e * self.timestep
        D = self.Kd * (e - self.lastError)
        result = P + self.I + D
        self.lastError = e

        # Scale down the twist if its norm is more than the maximum value
        resultNorm = np.linalg.norm(result)
        if resultNorm > self.maximumValue:
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result



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



def main(args=None):
    rclpy.init(args=args)
    robot_state_estimator = RobotStateEstimator()
    waypoint_square = np.array([[0.0,0.0,0.0], 
                         [1.0,0.0,0.0],
                         [1.0,0.0,np.pi/2],
                         [1.0,1.0, np.pi/2],
                         [1.0,1.0, np.pi],
                         [0.0,1.0, np.pi],
                         [0.0,1.0,1.5 * np.pi],
                         [0.0,0.0,1.5*np.pi],
                         [0.0,0.0,0],
                         [1.0,0.0,0.0],
                         [1.0,0.0,np.pi/2],
                         [1.0,1.0, np.pi/2],
                         [1.0,1.0, np.pi],
                         [0.0,1.0, np.pi],
                         [0.0,1.0,1.5 * np.pi],
                         [0.0,0.0,1.5*np.pi],
                         [0.0,0.0,0]])

    waypoint_octagon = np.array([
        [0.0,0.0,0.0],
        [0.414, 0.0,0.0],
        [0.414, 0.0,np.pi/4],
        [0.707, 0.293,np.pi/4],
        [0.707, 0.293, np.pi/2],
        [0.707, 0.707, np.pi/2],
        [0.707, 0.707, 3.0/4.0 * np.pi],
        [0.414, 1.0, 3.0/4.0 * np.pi],
        [0.414, 1.0, np.pi],
        [0.0, 1.0, np.pi],
        [0.0, 1.0, 5.0 / 4.0 * np.pi],
        [-.293, .707, 5.0/4.0 * np.pi],
        [-.293, .707, 3.0/2.0 * np.pi],
        [-.293, .293, 3.0/2.0 * np.pi],
        [-.293, .293, 7.0/4.0 * np.pi],
        [0.0, 0.0, 7.0/4.0 * np.pi],
        [0.0,0.0,0.0]
    ])
    
    


    # init pid controller
    pid = PIDcontroller(0.03,0.01,0.05)
    current_state = robot_state_estimator.current_state[:3]
    rclpy.spin_once(robot_state_estimator)


    for wp in waypoint_octagon:
        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)

        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.05): # check the error between current state and current way point
            # calculate the current twist

            update_value = pid.update(current_state)
            # publish the twist
            pid.publisher_.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)

            current_state = robot_state_estimator.update_state(update_value) 
            # if len(current_state) == 7:
            #     pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
            #     raise Exception("ERROR")
            current_state = current_state[:3]
            
            rclpy.spin_once(robot_state_estimator)



    print(robot_state_estimator.kf.P)
    # stop the car and exit
    pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

    pid.destroy_node()
    rclpy.shutdown()

   
if __name__ == '__main__':
    main()