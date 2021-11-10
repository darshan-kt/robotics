class PowerOnRobot(state):
	def __init__(self):
		State.__init__(self, outcomes= ['succeeded'])

	def execute(self, userdata):
		rospy.loginfo("Powering ON robot...")
		time.sleep(s)
		return 'succeeded'

class ButtonState(State):
	def __init__(self, button_state):
		State.__init__(self, outcomes=['succeeded','aborted','preempted'])
		self.button_state=button_state

	def execute(self, userdata):
		if self.button_state == 1:
		return 'succeeded'
	else:
		return 'aborted'

class OrderConfirmation(State):
	def __init__(self, user_confirmation):
		State.__init__(self, outcomes=['succeeded','aborted','preempted'])
		self.user_confirmation=user_confirmation

	def execute(self, userdata):
		time.sleep(2)
		if self.user_confirmation == 1:
		time.sleep(2)
		rospy.loginfo("Confirmation order...")
		time.sleep(2)
		rospy.loginfo("Order confirmed...")
		return 'succeeded'
	else:
		return 'preempted'

class SpeakOut(State):
	def __init__(self,chef_confirmation):
		State.__init__(self, outcomes=['succeeded','aborted','preempted'])
		self.chef_confirmation=chef_confirmation

	def execute(self, userdata):
		sleep(1)
		rospy.loginfo ("Please confirm the order")
		sleep(5)
		if self.chef_confirmation == 1:
		return 'succeeded'
	else:
		return 'aborted'

move_base_state = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb,
exec_timeout=rospy.Duration(5.0), server_wait_timeout=rospy.Duration(10.0))

quaternions = list()
euler_angles = (pi/2, pi, 3*pi/2, 0)
for angle in euler_angles:
	q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
	q = Quaternion(*q_angle)
	quaternions.append(q)

# Create a list to hold the waypoint poses
self.waypoints = list()
self.waypoints.append(Pose(Point(0.0, 0.0, 0.0), quaternions[3]))
self.waypoints.append(Pose(Point(-1.0, -1.5, 0.0), quaternions[0]))
self.waypoints.append(Pose(Point(1.5, 1.0, 0.0), quaternions[1]))
self.waypoints.append(Pose(Point(2.0, -2.0, 0.0), quaternions[1]))
room_locations = (('table', self.waypoints[0]),
('delivery_area', self.waypoints[1]),
('kitchen', self.waypoints[2]),('home', self.waypoints[3]))

self.room_locations = OrderedDict(room_locations)
nav_states = {}
for room in self.room_locations.iterkeys():
	nav_goal = MoveBaseGoal()
	nav_goal.target_pose.header.frame_id = 'map'
	nav_goal.target_pose.pose = self.room_locations[room]
	move_base_state = SimpleActionState('move_base', MoveBaseAction,
	goal=nav_goal, result_cb=self.move_base_result_cb,
	exec_timeout=rospy.Duration(5.0),
	server_wait_timeout=rospy.Duration(10.0))
	nav_states[room] = move_base_state