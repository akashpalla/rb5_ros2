"""
This file include code that control the robot motors
"""

import numpy as np
import csv
from megapi import MegaPi

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
        self.wheel_speed_multiplier = 10

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
        # if self.verbose:
        #     print()
        wheel_speeds = self.wheel_speed_multiplier * wheel_speeds
        print("Adjusted wheel speeds")
        print(wheel_speeds)

        # self.bot.motorRun(self.mfl,-wheel_speeds[0]) #FL
        # self.bot.motorRun(self.mfr,wheel_speeds[1]) #FR
        # self.bot.motorRun(self.mbl,-wheel_speeds[2]) #BL
        # self.bot.motorRun(self.mbr,wheel_speeds[3]) #BR




    

    def velocityToWheel(self, velocity):
        wheel_speeds = 1 / self.radius * np.dot(self.kinematics, velocity) * wheel_speeds
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
        dt = distance / fixed_robot_speed
        print(dt)
        wheel_speeds = self.velocityToWheel(np.array([fixed_robot_speed, 0, 0]))
        self.setFourMotorsNew(wheel_speeds)
        time.sleep(dt)


    def carWaypoint(self, filename):
        file = open(filename, "r"):
        line = file.readline()
        if line: # sanity check to make sure line has content
            curr_waypoint = np.atleast_2d(np.array([float(i) for i in line.split(',')])).T

        line = file.readline() # get "first" waypoint to travel to
        while line:
            next_waypoint = np.atleast_2d(np.array([float(i) for i in line.split(',')])).T
            waypoint_diff = next_waypoint - curr_waypoint
            





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
    mpi_ctrl.carStraightDistance(1)
        
    # mpi_ctrl.carStraight(30)
    # time.sleep(4)
    # mpi_ctrl.carSlide(30)
    # time.sleep(1)
    # mpi_ctrl.carRotate(30)
    # time.sleep(1)
    mpi_ctrl.carStop()
    mpi_ctrl.close()
