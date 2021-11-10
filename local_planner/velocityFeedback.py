#!/usr/bin/env python2
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped,TransformStamped
from std_msgs.msg import Float64, Int8,Duration

previous_msg=None
speed_lst=[]

def calculate_speed(msg):
    global previous_msg
    pub = rospy.Publisher('/speed', Float64, queue_size=50)
    if(previous_msg==None):
        previous_msg=msg
    else:
        x1=previous_msg.pose.position.x
        y1=previous_msg.pose.position.y
        x2=msg.pose.position.x
        y2=msg.pose.position.y

        t=msg.header.stamp-previous_msg.header.stamp
        previous_msg=msg
        time = (t.nsecs *10**-9)/1.0
        speed=np.sqrt((x2-x1)**2+(y2-y1)**2)/time
        
        speed_lst.append(speed)
        if(len(speed_lst)==11):
          speed_lst.pop(0)
        # print(len(speed_lst))
        pub_speed = sum(speed_lst)/len(speed_lst)
        if(pub_speed < 1.5):
          pub_speed=1.5
        pub.publish(pub_speed)

          





      
if __name__== "__main__":
  rospy.init_node( 'ndt_speed', anonymous=True)
  rospy.Subscriber('/ndt_pose', PoseStamped, calculate_speed)
  rospy.spin()
