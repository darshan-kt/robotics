'''
Developed by Darshan KT
The aim of this code is to convert bicycle model motion equations into useful code format and visualize changing poses over time 
'''

from pylab import *
import math

delta = 0.707 #rad  (45 degree)
Time_dur = 0.3  #sec
v = 1.00 #m/s
l = 2.00 #m           If increase wheelbase length, model/robot creates larger radius circle
time = arange(0.0, 10, 0.15)  #Dividing 10s into 0.15 timestamps each
x1 = y1 = theta1 = 0.0    #Initial pose value
X_pose = []
Y_pose = []
orintation = []
timestamps = []

#Updation function for each timestamps
def poseUpdate(x,y,z):
	# Bicycle model motion eqautions
	theta2 = theta1 + ((v/l) * math.tan(delta) * Time_dur)  #Time_dur = T2-T1
	x2 = x1 + ((l/math.tan(delta) * (math.sin(theta2)- math.sin(theta1))))
	y2 = y1 + ((l/math.tan(delta) * (math.cos(theta1) - math.cos(theta2))))
	return x2, y2, theta2


#Looping conditons for updation of poses
for timeSteps in time:
	X_pose.append(x1)
	Y_pose.append(y1)
	orintation.append(theta1)
	timestamps.append(timeSteps)
	a,b,c = poseUpdate(x1, y1, theta1)
	print(a,b,c)
	x1, y1, theta1 = a, b, c

plot(X_pose, Y_pose, 'r.')     #This graph indicates X, Y pose of robot at each timestamps, because of giving constant speed and steering angle robot model makes circlar motion
xlabel('x-axis')
ylabel('y-axis')
title('Motion of bicycle model in xy-plane')
grid(True)
show()
