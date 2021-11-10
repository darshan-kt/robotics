#!/usr/bin/env python
import tf
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64



def Orient_try(msg,pub):
    quaternion = (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw_p = np.rad2deg(euler[2])
    if (yaw_p<0):
        yaw_p+=360
    pub.publish(euler[2])
    print("yaw_p", yaw_p)
    


if __name__== "__main__":

    rospy.init_node( 'quart', anonymous=True) 
    pub = rospy.Publisher('/orientation', Float64, queue_size=10)
    rospy.Subscriber('/ndt_pose', PoseStamped,Orient_try,pub)
    rospy.spin()
    