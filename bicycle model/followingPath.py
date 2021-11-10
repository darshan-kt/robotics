from pylab import *
import math

Time_dur = 0.2 #sec
# v = 2.00  #m/s
l = 2.00 #m           If increase wheelbase length, model/robot creates larger radius circle
time = arange(0.0, 10, 0.25)  #Dividing 10s into 0.21 timestamps each
x1, y1, theta1 = 8, 5, 1.57   #Initial pose value
X_pose = []
Y_pose = []
orintation = []

vel = 0.0
angle = 0.0
velocity = []
steering =[]

def velDelta_cal(x,y,theta):
	d =   #maintain dist
	dt = 3
	Kv =
	Ki = 
	Kh =
	erorr = math.sqrt((x - x1)**2 + (y - y1)**2) - d
	v = Kv * erorr +  (Ki * erorr * dt)
	theta = math.atan((y - y1)/(x - x1))

	t = sin(theta - theta1)   
	s = cos(theta - theta1) 
	angular_diff = math.atan2(t,s)

	delta = Kh * angular_diff
	return v, delta

#Updation function for each timestamps
def poseUpdate(x1, y1, theta1, v, delta):
    # Bicycle model motion eqautions
    theta2 = theta1 + ((v/l) * math.tan(delta) * Time_dur)  #Time_dur = T2-T1
    x2 = x1 + ((l/math.tan(delta) * (math.sin(theta2)- math.sin(theta1))))
    y2 = y1 + ((l/math.tan(delta) * (math.cos(theta1) - math.cos(theta2))))
    # print(x2,y2,theta2)
    return x2, y2, theta2

waypoints = [(),(),().......]  #need to collet waypoints as set of x,y poses. 


for pose in waypoints:     
'''
#In pure pursuit waypoints itself consider as goal points. 
But they r not cosider each waypoint as goal points instead skip some pose points and consider immidiat next as goal point.
Example skips 10 points and consider 11th one as goal point, this skipping points will also depends on maintain distance.
'''

	xg,yg = pose       #xg,yg are target goal pose values

    X_pose.append(x1)
    Y_pose.append(y1)
    orintation.append(theta1)
    steering.append(angle)

    v, delta = velDelta_cal(xg, yg, theta1)
    a,b,c = poseUpdate(x1,y1,theta1,v,delta)   
    x1, y1, theta1 = a, b, c