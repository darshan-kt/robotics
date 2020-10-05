#!/usr/bin/env python
"""
We will see

"""

import rospy
from std_msgs.msg import Int64, Float64MultiArray, Float64, Float32MultiArray, Bool

from PID_class import *
# from camera_radar_msg.msg import fused_data
# from camera_radar_msg.msg import fused_data_
import time
from dynamic_parameters.cfg import pid_paramsConfig


class Controls(object):
    def __init__(self):

        rospy.loginfo("Controls class initialized")
        # Initialize ROS node
        rospy.init_node('test_control', anonymous=True)

        # PID Parameters
        self.Accel1Param = [1.5, 0.1, 0.0]	#old values 1.07, 0.022, 0    TUNABLE PID VALUES 
        self.Accel1Param = [self.Kp, self.Ki, self.Kd]
        print("Parameters", self.Accel1Param)
        self.SteerParam = [0.0000, 0.000, 0.000]
        self.Brake2Param = [2, 0.0, 0.0]

        # Proportional Factor       
        self.Accel1Fact = 1.0
        self.SteerFact = 1.0
        self.Brake2Fcat = 1.0

        # Constrain Parameter           Maximum limit of motor
        self.Accel1Constrain = 70.0  # 0.0
        self.SteerConstrain = 1000.0
        self.Brake2Constrain = 80.0  # 100

        # Objects for PID class
        self.LK1 = PID(self.Accel1Fact, self.Accel1Param, self.Accel1Constrain)
        self.LK2 = PID(self.Accel2Fact, self.Accel2Param, self.Accel2Constrain)
        self.steer = PID(self.SteerFact, self.SteerParam, self.SteerConstrain)
        self.Brake1 = PID(self.Brake1Fact, self.Brake1Param, self.Brake1Constrain)
        self.Brake2 = PID(self.Brake2Fcat, self.Brake2Param, self.Brake2Constrain)

        # Errors
        self.Accel1Error = 0.0
        self.Accel2Error = 0.0
        self.SteerError = 0.0
        self.Brake1Error = 0.0
        self.Brake2Error = 0.0

        # Motor Speed      Motor speed depending on motor specification for increaing torque rate
        self.BrakeMotorSpeed = 2
        self.SteerMotorSpeed = 1

        # Speed related
        self.ObjectPresent = False
        self.ReferenceSpeed = 0
        self.BrakeAngle = 0.0
        self.CurrentVelocity = 0.0
        self.EmergencyBrakeMotorSpeed = 1
        self.EmergencyBrakeAngle = 80

        # Limiting Variables
        self.AccelMax = 80
        self.NormalAccel = 1
        self.ThrottlePos = 1

        self.Brake = Float32MultiArray()
        self.Accel = Float32MultiArray()
        self.Steer = Float32MultiArray()
        self.BrakeEmergency = Float32MultiArray()

        if not rospy.is_shutdown():
            # Subscribers
            rospy.Subscriber("/Embedded/Inputs", Float32MultiArray, self.get_data, queue_size=1)    #Odometry
            rospy.Subscriber("/manual_speed", Float64, self.call_manualSPeed)                       #manual speed
            rospy.Subscriber("/emergencyFlag", Bool, self.get_emergency, queue_size=1)              #Emergency braking 

            # Publisher
            self.SteerPub = rospy.Publisher('/Steering_Topic', Float32MultiArray, queue_size=1)  # For steering motor
            self.BrakePub = rospy.Publisher('/Embedded/Brake_Topic', Float32MultiArray, queue_size=1)  # For brake motor
            self.AccelPub = rospy.Publisher('/Embedded/Accel_Topic', Float32MultiArray, queue_size=1)  # For accelerator motor
            self.SpeedPub = rospy.Publisher('/OBD', Float64, queue_size=1)  # For accelerator motor
            self.rate = rospy.Rate(100)
            rospy.spin()                                    

    def get_vel(self, msg):

        if (self.ObjectPresent == False):
            self.ReferenceSpeed = (msg.data * 3.6)  # Data is converted from m/s to km/h
        # print("Velocity Status:",self.ReferenceSpeed, self.CurrentVelocity)

    # self.StopBoardPresentdDistance = msg.data[3]
    def call_manualSPeed(self, msg):
        self.ReferenceSpeed = msg.data
        print('manual velocity given', self.ReferenceSpeed)

    def callback(config, level):
        print("Kp = ", config.Kp, "\tKi = ", config.Ki, "\tKd = ", config.Kd)
        rospy.set_param('PID', [config.Kp, config.Ki, config.Kd])

        return config


    def get_MPCspeed(self, msg):
        if self.ObjectPresent == False:
            self.ReferenceSpeed = (msg.data)  # Data is converted from m/s to km/h
        #	print("Velocity Status:",self.ReferenceSpeed, self.CurrentVelocity)
        else:
            self.ReferenceSpeed = 0

    def get_MPCsteer(self, msg):
        self.RadianAngle = msg.data

    def get_emergency(self, msg):
        # self.ReferenceSpeed = 0
        self.ObjectPresent = msg.data
        if self.ObjectPresent == True:
            self.ReferenceSpeed = 0

    def get_data(self, data):
        self.CurrentVelocity = data.data[21]     #Odometry data


    def get_obj_dis(self, obj_dis):
        # self.ObjectDistance = obj_dis.data
        self.STOPBoardDistance = obj_dis.data

    def convertor(self):
        if (self.RadianAngle != self.PrevRadianAngle):
            #	self.MotorAngle = self.RadianAngle

            self.BodyAngle = self.RadianAngle * self.RadianFactor
            self.MotorAngle = self.BodyAngle * self.MotorToBodyFactor
            # Use this for normal system
            self.Steer.data = [self.MotorAngle, 1.65]
            # Use this for Closed loop system
            #	self.Steer.data = [self.RadianAngle, 1.8]

            self.SteerPub.publish(self.Steer)
            self.PrevRadianAngle = self.RadianAngle
            print("Motor Angle: ", self.MotorAngle)
            print("Body Angle: ", self.BodyAngle)
            time.sleep(0.1)

    def EmergencyBraking(self):

        self.BrakeEmergency.data = [self.EmergencyBrakeAngle, self.EmergencyBrakeMotorSpeed]
        self.BrakePub.publish(self.BrakeEmergency)
        time.sleep(0.1)

        self.Accel.data = [1, 1]
        self.AccelPub.publish(self.Accel)
        time.sleep(0.2)

    def errorcheck(self):
        data1 = Float64MultiArray()
        data1.data = [self.CurrentSteeringAngle]
        self.SteerRfbPub.publish(data1)

    # check position error for Accel

    # check position error for Brake

    # check position error for Steering

    # Data logger

    def cruizer(self):
        if (self.ReferenceSpeed == 99):
            return 0

        elif (self.ReferenceSpeed >= 100):           #Release all and give control to manual
            self.BrakeAngle = 0
            self.BrakeMotorSpeed = 1
            self.Brake.data = [self.BrakeAngle, self.BrakeMotorSpeed]
            self.BrakePub.publish(self.Brake)
            time.sleep(0.05)

            self.AccelMotorSpeed = 0
            self.NormalAccel = 1
            self.Accel.data = [self.NormalAccel, self.AccelMotorSpeed]
            self.AccelPub.publish(self.Accel)
            time.sleep(0.05)
            return 0
        '''	
		
							if (obstacle_dis/StopSign)
									|
					-----------------------------------------------------------------
					|                                                               |
				YES |                                                            NO |
					|                                                               |
					|                                                               |
					V                                                               |  
		-------------------------------------------                                 |
		|                       |                  |                                |
		| obstacle_dis          | obstacle_dis     |                                |
		|  < 4                  |    4 - 20        |                                |
		V                       V                  V                                | 
	Emergency braking     Soft braking         Do Nothing                           | 
							  PID                                                   |
																					V
														------------------------------------------------
														|                           |                   |
														|                           |                   |
														|                           |                   |
												current speed <               current speed ==        current speed > refernce speed +2     
												reference speed - 2             reference speed +- 2    |
														|                           |                   |
														|                           |                   |
														V                           V                   V
													accelerate & Release brake    do nothing     brake & deaccelrate
															PID
		'''
        if self.ObjectPresent:
            print("Going to Emergency ")
            self.EmergencyBraking()

        else:


            if self.ReferenceSpeed == 0:
                self.BrakeAngle = 80        #press brake
                self.BrakeMotorSpeed = 1
                self.Brake.data = [self.BrakeAngle, self.BrakeMotorSpeed]             
                self.BrakePub.publish(self.Brake)
                print("Initial brake Angle = ", self.Brake)
                time.sleep(0.6)

                self.AccelMotorSpeed = 1
                self.NormalAccel = 1
                self.Accel.data = [self.NormalAccel, 1]
                self.AccelPub.publish(self.Accel)
                print("Initial Accel Angel = ", self.Accel)
                time.sleep(0.05)

            # Condition 1 : When Current velocity is smaller than Reference Speed

            elif self.CurrentVelocity <= self.ReferenceSpeed:
                self.AccelError = self.ReferenceSpeed - self.CurrentVelocity
                # self.NormalAccel = 25 + self.LK1.calculate(self.AccelError)
                self.NormalAccel = self.LK1.calculate(self.AccelError)             #Finding out Error using library
                self.Accel.data = [self.NormalAccel, self.AccelMotorSpeed]         #Pass accelaration as accelearation with motorSpeed
                self.AccelPub.publish(self.Accel)    #Publish error
                print("Curr < referr", self.NormalAccel)
                time.sleep(0.05)

            # Condition 2: When need to reduce speed            
            elif self.CurrentVelocity > (self.ReferenceSpeed + 1):
                #				print("Braking ")
                self.AccelError = self.CurrentVelocity - self.ReferenceSpeed
                # self.NormalAccel = 25 + self.LK1.calculate(self.AccelError)
                self.NormalAccel = self.LK1.calculate(self.AccelError)
                self.Accel.data = [self.NormalAccel, self.AccelMotorSpeed]
                self.AccelPub.publish(self.Accel)    #Publish error
                print("Normal_accel after reducing", self.NormalAccel)
                time.sleep(0.05)

                #Brake PID calculations
                self.Brake2Error = self.CurrentVelocity - self.ReferenceSpeed
                self.BrakeAngle = 30 + (self.Brake2.calculate(self.Brake2Error))
                # self.BrakeAngle = (int(self.BrakeAngle/4)*4)
                self.BrakeMotorSpeed = 2
                #	print("Brakeeeeeeeeeeeeeeeee", self.BrakeAngle)

                if self.BrakeAngle > 70:
                    self.BrakeMotorSpeed = 3
                    self.BrakeAngle = 80

                self.Accel.data = [self.NormalAccel, self.AccelMotorSpeed]
                self.AccelPub.publish(self.Accel)
                time.sleep(0.05)

                # print("Brake Actuating", self.BrakeAngle)
                self.Brake.data = [self.BrakeAngle, self.BrakeMotorSpeed]
                self.BrakePub.publish(self.Brake)
                time.sleep(0.05)

            print(self.Brake, self.NormalAccel)


def main(args):
    try:
        control = Controls()
        while not rospy.is_shutdown():
            control.cruizer()
        # control.convertor()
        # control.errorcheck()
        # time.sleep(0.05)
    except KeyboardInterrupt():
        print("Shutting down..")
        rospy.shutdown()

    rospy.shutdown()


if __name__ == '__main__':
    main()
    rospy.init_node("pid_parameters", anonymous = False)   # Node name will appear in the GUI

    rospy.spin()
