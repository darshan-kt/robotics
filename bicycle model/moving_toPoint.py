#!/usr/bin/env python
'''
Developed by Darshan KT
Objective: Perform moving to a goal point operation using bicycle model
'''


from pylab import *
import matplotlib.pyplot as plt
import math
import numpy

Time_dur = 0.3 #sec
l = 2.00 #m           If increase wheelbase length, model/robot creates larger radius circle
time = arange(0.0, 10, 0.20)  #Dividing 10s into 0.21 timestamps each
x1, y1, theta1 = 8, 5, 1.57   #Initial pose value
x, y = 5, 5
X_pose = []
Y_pose = []
orintation = []
timestamps = []
vel = 0.0
angle = 0.0
velocity = []
steering =[]


def findVelAndTheta(x1, y1, theta1):
    Kv = 0.5
    Kh = 4
    # Formulas used for finding vel and steering angle to move model to a particular point
    v = Kv * (math.sqrt((x-x1)**2) + (y-y1)**2)
    try:
        theta = math.atan((y - y1) / (x - x1))
    except ZeroDivisionError:
        theta = 0
    delta = Kh * (theta - theta1)
    return v, delta

#Updation function for each timestamps
def poseUpdate(v, delta):
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
    velocity.append(vel)
    steering.append(angle)

    vel, angle = findVelAndTheta(x1, y1, theta1)
    a,b,c = poseUpdate(vel, angle)   
    print(vel, angle, a, b)  #when the robot model comes nearer to the goal point velocity of model automatically comes to zero.
    x1, y1, theta1 = a, b, c


plt.subplot(3, 1, 1)
plt.plot(X_pose, Y_pose, 'o-')
plt.xlabel('x-axis')
plt.ylabel('y-axis')
plt.title('Bicycle model motion estimations')

plt.subplot(3, 1, 2)
plt.plot(time, velocity, 'o-')
plt.ylabel('Change in velocity')

plt.subplot(3, 1, 3)
plt.plot(time, steering, '.-')
plt.xlabel('time (s)')
plt.ylabel('change in st. angle')
plt.show()