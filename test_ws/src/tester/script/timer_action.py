#! /usr/bin/env python
import rospy
import time
import actionlib
from tester.msg import TimerAction, TimerGoal, TimerResult, TimerFeedback

def do_timer(goal):
	start_time = time.time()
	time.sleep(goal.time_to_wait.to_sec())
	result = TimerResult()                           #create a result object
	result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)  #Assign data field to result object
	result.updates_sent = 0
	server.set_succeeded(result)   #set_succeeded(): simillar to publisher to publish result through server


rospy.init_node('timer_action_server')
server = actionlib.SimpleActionServer('timer', TimerAction, do_timer, False)   #syntax for server, 'False' defines server is not autostarted
server.start()           #start() function used to start server
rospy.spin()

