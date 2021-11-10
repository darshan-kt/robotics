#!/usr/bin/env python2
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
import rospy
import numpy as np
import math


def calculate_speed(input,pub):
    x=input.twist.linear.x
    y=input.twist.linear.y
    speed=0
    # print(x)
    if(x!=x):
        x=0
    if(y!=y):
        y=0
    speed=np.sqrt(x*x + y*y)
    # print(speed)
    pub.publish(speed)
    

if __name__== "__main__":
    print('Started publishing in /rtkSpeed')
    rospy.init_node( 'speed', anonymous=True) 
    pub = rospy.Publisher('/rtkSpeed', Float64, queue_size=10)
    sub_compasss = rospy.Subscriber('/vel', TwistStamped, calculate_speed,pub)
    #mob_compasss = rospy.Subscriber('/mob_compass_angle', Float64, mob_compass,pub)
    
    rospy.spin()