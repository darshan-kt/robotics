#! /usr/bin/env python
import rospy
import actionlib
from tester.msg import TimerAction, TimerGoal, TimerResult
rospy.init_node('timer_action_client')
client = actionlib.SimpleActionClient('timer', TimerAction) #client syntax using actionlib package
client.wait_for_server()   #wait_for_server(): this function is used to wait for server activation
goal = TimerGoal()         # Create goal object for TimerGoal class data type
goal.time_to_wait = rospy.Duration.from_sec(5.0)   #fill the goal atrribute defined in action def file
client.send_goal(goal)           # Simlilar publisher to publish goal through client
client.wait_for_result()         # This functions alive the client node untill the result from the server
print('Time elapsed: %f'%(client.get_result().time_elapsed.to_sec()))   #accesing the result using get_result() with data member/attribute defined in action file