"""
This file include code that control the robot motors
"""

import numpy as np
import csv
from megapi import MegaPi
import math

# You need to tune these numbers, if needed, to find the correct port for each wheel
# The range should be integers from 0 to 14
MFR = 2     # port for motor front right
MBL = 3     # port for motor back left
MBR = 10    # port for motor back right
MFL = 11    # port for motor front left


class MegaPiController:
    def __init__(self, port='/dev/ttyUSB0', verbose=True):
        self.port = port
        self.verbose = verbose
        if verbose:
            self.printConfiguration()
        self.bot = MegaPi()
        self.bot.start(port=port)
        self.mfr = MFR  # port for motor front right
        self.mbl = MBL  # port for motor back left
        self.mbr = MBR  # port for motor back right
        self.mfl = MFL  # port for motor front left   
        
        #meters
        self.lx = .06747
        self.ly = .05635
        self.radius = .03016
        self.drive_multiplier = 10
        self.turn_multiplier = 12

        self.kinematics = np.array([
            [1, -1, -1 * (self.lx + self.ly)],
            [1, 1, (self.lx + self.ly)],
            [1, 1, -1 * (self.lx + self.ly)],
            [1, -1, (self.lx + self.ly)]
        ])




    
    def printConfiguration(self):
        print('MegaPiController:')
        print("Communication Port:" + repr(self.port))
        print("Motor ports: MFR: " + repr(MFR) +
              " MBL: " + repr(MBL) + 
              " MBR: " + repr(MBR) + 
              " MFL: " + repr(MFL))


    def setFourMotors(self, vfl=0, vfr=0, vbl=0, vbr=0):
        if self.verbose:
            print("Set Motors: vfl: " + repr(int(round(vfl,0))) + 
                  " vfr: " + repr(int(round(vfr,0))) +
                  " vbl: " + repr(int(round(vbl,0))) +
                  " vbr: " + repr(int(round(vbr,0))))
        self.bot.motorRun(self.mfl,-vfl)
        self.bot.motorRun(self.mfr,vfr)
        self.bot.motorRun(self.mbl,-vbl)
        self.bot.motorRun(self.mbr,vbr)
    
    def setFourMotorsNew(self, wheel_speeds):
        
        wheel_speeds = wheel_speeds.astype(int)
        # wheel_speeds = np.clip(wheel_speeds, a_min=-50, a_max=50)
        if self.verbose:
            print("Set Motors: vfl: " + str(wheel_speeds[0]) + 
                  " vfr: " + str(wheel_speeds[1]) +
                  " vbl: " + str(wheel_speeds[2]) +
                  " vbr: " + str(wheel_speeds[3])
                  )
        

        self.bot.motorRun(self.mfl,-wheel_speeds[0]) #FL
        self.bot.motorRun(self.mfr,wheel_speeds[1]) #FR
        self.bot.motorRun(self.mbl,-wheel_speeds[2]) #BL
        self.bot.motorRun(self.mbr,wheel_speeds[3]) #BR



    def velocityToWheel(self, velocity):
        wheel_speeds = 1 / self.radius * np.dot(self.kinematics, velocity) 
        print("Wheel speeds")
        print(wheel_speeds)
        # print("------------------")
        return wheel_speeds
    


    # The actual motor signal need to be tuned as well.
    # The motor signal can be larger than 50, but you may not want to go too large (e.g. 100 or -100)
    def carStop(self):
        if self.verbose:
            print("CAR STOP:")
        self.setFourMotors()


    def carStraightDistance(self, distance):
        fixed_robot_speed = .1 #m/s

        if distance < 0:
            fixed_robot_speed = -1 * fixed_robot_speed

        dt = distance / fixed_robot_speed
        wheel_speeds = self.velocityToWheel(np.array([-1 * fixed_robot_speed, 0, 0]))
        self.setFourMotorsNew(wheel_speeds * self.drive_multiplier)
        print(dt)
        time.sleep(dt)
        self.carStop()

    def carTurnInPlace(self, angle):
        fixed_robot_turn_speed = 1.2 # rad / s

        if angle < 0:
            fixed_robot_turn_speed = -1 * fixed_robot_turn_speed

        dt = angle / fixed_robot_turn_speed
        wheel_speeds = self.velocityToWheel(np.array([0, 0, fixed_robot_turn_speed]))
        self.setFourMotorsNew(wheel_speeds * self.turn_multiplier)
        time.sleep(dt)
        self.carStop()

    def generateTrajectory(distance, cruise_velocity, acceleration, dt):
        max_velocity = math.sqrt(distance * acceleration)
        cruise_velocity = min(max_velocity, cruise_velocity)
        
        time_to_accel = cruise_velocity/ acceleration
        distance_after_accel = .5 * acceleration * math.pow(time_to_accel,2)
        cruise_distance = distance - 2* distance_after_accel
        time_at_vcruise = cruise_distance/ cruise_velocity

        running_time_ms = 0
        current_pos = 0
        current_vel = 0
        current_accel = 0

        t_list = []
        x_list = []
        v_list = []
        a_list = []

        while(running_time_ms <= (2*time_to_accel + time_at_vcruise)*1000):

            if(running_time_ms < time_to_accel * 1000):
                current_accel = acceleration
            elif(running_time_ms < (time_to_accel + time_at_vcruise)*1000):
                current_accel = 0
            else:
                current_accel = -1 * acceleration
        
            current_vel += current_accel * dt
            current_vel = min(cruise_velocity, current_vel)

            current_pos += current_vel* dt + .5 * current_accel * dt * dt

            print("t: {:.2f} x: {:.2f} v:{:.2f} a: {:.2f}".format(running_time_ms/1000,current_pos,current_vel,current_accel))

            t_list.append(running_time_ms/1000)
            x_list.append(current_pos)
            v_list.append(current_vel)
            a_list.append(current_accel)

            running_time_ms += 1000*dt

        return (t_list, x_list)

    # def carWaypoint(self, filename):
    #     file = open(filename, "r")
    #     line = file.readline()
    #     if line: # sanity check to make sure line has content
    #         curr_waypoint = np.atleast_2d(np.array([float(i) for i in line.split(',')])).T

    #     line = file.readline() # get "first" waypoint to travel to
    #     while line:
    #         next_waypoint = np.atleast_2d(np.array([float(i) for i in line.split(',')])).T
    #         waypoint_diff = next_waypoint - curr_waypoint
            



    def carStraight(self, speed):
        if self.verbose:
            print("CAR STRAIGHT:")
        self.setFourMotors(-speed, speed, -speed, speed)


    def carRotate(self, speed):
        if self.verbose:
            print("CAR ROTATE:")
        self.setFourMotors(speed, speed, speed, speed)


    def carSlide(self, speed):
        if self.verbose:
            print("CAR SLIDE:")
        self.setFourMotors(speed, speed, -speed, -speed)

    
    def carMixed(self, v_straight, v_rotate, v_slide):
        if self.verbose:
            print("CAR MIXED")
        self.setFourMotors(
            v_rotate-v_straight+v_slide,
            v_rotate+v_straight+v_slide,
            v_rotate-v_straight-v_slide,
            v_rotate+v_straight-v_slide
        )
    
    def close(self):
        self.bot.close()
        self.bot.exit()


if __name__ == "__main__":
    import time
    mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=True)  
    time.sleep(1)

    unit_distance = 0.4

    #start
    mpi_ctrl.carStraightDistance(-unit_distance)
    time.sleep(0.5)

    #waypoint 1
    mpi_ctrl.carTurnInPlace(1.57)
    mpi_ctrl.carStraightDistance(unit_distance)
    time.sleep(0.5)

    #waypoint 2
    mpi_ctrl.carTurnInPlace(-1.57)
    mpi_ctrl.carStraightDistance(-unit_distance)
    time.sleep(0.5)

    #waypoint 3
    mpi_ctrl.carTurnInPlace(-1.57)
    mpi_ctrl.carStraightDistance(-unit_distance)
    time.sleep(0.5)

    #waypoint 4
    mpi_ctrl.carTurnInPlace(1.57/2)
    mpi_ctrl.carStraightDistance(math.sqrt(2 * (unit_distance ** 2)))
    time.sleep(0.5)

    #waypoint 5
    mpi_ctrl.carStraightDistance(math.sqrt(2 * (unit_distance ** 2)))
    time.sleep(0.5)

    print("Finished Routine")
    #finish
    mpi_ctrl.carStop()
    mpi_ctrl.close()
