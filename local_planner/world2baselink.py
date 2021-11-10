#!/usr/bin/env python
import rospy
import tf
import tf2_ros
# import turtlesim.msg
from geometry_msgs.msg import PoseStamped,TransformStamped

pub = rospy.Publisher('/ndt_pose', PoseStamped, queue_size=10)
br = tf2_ros.transform_broadcaster.TransformBroadcaster()
def pose_callback(msg):
    
    # br.sendTransform((0, 0, 0),
    #                  (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w),
    #                  rospy.Time.now(),
    #                  "base_link",
    #                  "world")
    # pose_new = msg
    
    # pub.publish(pose_new)
    # print(1)
    t= TransformStamped()
    t.header.frame_id = "world"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = "base_link"
    t.transform.translation.x= msg.pose.position.x
    t.transform.translation.y= msg.pose.position.y
    t.transform.translation.z= msg.pose.position.z

    t.transform.rotation.x= msg.pose.orientation.x
    t.transform.rotation.y= msg.pose.orientation.y
    t.transform.rotation.z= msg.pose.orientation.z
    t.transform.rotation.w= msg.pose.orientation.w
    br.sendTransform(t)

    msg.header.frame_id = "world"
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)






if __name__ == '__main__':
    rospy.init_node('truck_tf_broadcaster')
    rospy.Subscriber('/truck/gps/pose',PoseStamped,pose_callback)
                     
    rospy.spin()