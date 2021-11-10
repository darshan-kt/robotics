#!/usr/bin/env python
'''
Developed by Darshan KT
Objective: Perform line following technique using bicycle model
'''

import matplotlib.pyplot as plt
import numpy as np
import math
from pylab import *


'''
Straight line information:
1. Calculate the slope and y-intercept point from the standard straight line equation
https://www.youtube.com/watch?v=vqqu0QYBGJ8

2. Sketch the line using given slope and y-intercept
https://www.youtube.com/watch?v=SMfA3rjRvN8

3. Draw in python
https://scriptverse.academy/tutorials/python-matplotlib-plot-straight-line.html
'''

Time_dur = 0.2 #sec
v = 2.00  #m/s
l = 2.00 #m           If increase wheelbase length, model/robot creates larger radius circle
time = arange(0.0, 10, 0.25)  #Dividing 10s into 0.21 timestamps each
x1, y1, theta1 = 8, 5, 1.57   #Initial pose value
X_pose = []
Y_pose = []
orintation = []
timestamps = []

vel = 0.0
angle = 0.0
velocity = []
steering =[]

a, b, c = 1.0, -2.0, 4.0

def calSlope():
	# ax + by + c = 0    Standard equation for straight line
	# y = (-a/b)*x + (-c/b)     Convert into Equation of line format to calculate slope and y-intercept  i.e, y = mx + b
	slope = float(-a/b)
	y_intercept = float(-c/b)
	print("Equation for straight line is: y = ", slope ,'* x +', y_intercept)
	return slope, y_intercept

def DrawLine(slope, y_intercept):
	m = slope
	b = y_intercept
	x = np.linspace(-1,10,100)
	y = m*x + b

	plt.plot(x, y, '--r', label='y=0.5x+2')
	plt.title('Graph of y=0.5x+2')
	plt.xlabel('x', color='#1C2833')
	plt.ylabel('y', color='#1C2833')
	# plt.legend(loc='upper left')
	plt.grid()
	plt.show()

# r, t = calSlope()
# DrawLine(r,t)

def calDelta(x,y,theta1):
	Kd = 0.5
	Kh = 1
	f = np.array([a,b,c])
	g = np.array([x,y,1])
	h = math.sqrt(a**2 + b**2)
	dist = np.dot(f, g)/h
	alphaD = - Kd * dist
	theta = math.atan(-a/b)    #theta is target angle and theta1 is current state angle

	t = sin(theta - theta1)   
	s = cos(theta - theta1) 
	angular_diff = math.atan2(t,s) # calculation for smallest angular difference on circle or angular difference between 2 angles

	print(theta, theta1, angular_diff)
	alphaH = Kh *  angular_diff
	delta = alphaD + alphaH
	# print(delta)
	return delta

#Updation function for each timestamps
def poseUpdate(delta):
    # Bicycle model motion eqautions
    theta2 = theta1 + ((v/l) * math.tan(delta) * Time_dur)  #Time_dur = T2-T1
    x2 = x1 + ((l/math.tan(delta) * (math.sin(theta2)- math.sin(theta1))))
    y2 = y1 + ((l/math.tan(delta) * (math.cos(theta1) - math.cos(theta2))))
    # print(x2,y2,theta2)
    return x2, y2, theta2


#Looping conditons for updation of poses
for timeSteps in time:
    X_pose.append(x1)
    Y_pose.append(y1)
    orintation.append(theta1)
    timestamps.append(timeSteps)
    # velocity.append(vel)
    steering.append(angle)

    angle = calDelta(x1, y1, theta1)
    a,b,c = poseUpdate(angle)   
    # print(angle, a, b, c)  #when the robot model comes nearer to the goal point velocity of model automatically comes to zero.
    x1, y1, theta1 = a, b, c

# plot(time, steering)
plot(X_pose, Y_pose, 'r.')     #This graph indicates X, Y pose of robot at each timestamps, because of giving constant speed and steering angle robot model makes circlar motion
xlabel('x-axis')
ylabel('y-axis')
title('Motion of bicycle model in xy-plane')
grid(True)
show()


# plt.subplot(2, 1, 1)
# plt.plot(X_pose, Y_pose, 'o-')
# plt.xlabel('x-axis')
# plt.ylabel('y-axis')
# plt.title('Bicycle model motion estimations')
# t, s = DrawLine()
# # plt.subplot(2, 1, 1)
# plt.plot(t, s, '--r')
# plt.title('Graph of y=0.5x+2')
# plt.xlabel('x', color='#1C2833')
# plt.ylabel('y', color='#1C2833')
# plt.show()
