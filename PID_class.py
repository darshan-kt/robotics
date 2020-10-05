#!/usr/bin/env python
import rospy
import time

stable = []
stable_turn = []
runtime = int(round(time.time() * 1000))


class PID:
	"""
	Class for PID calculation
	Inputs: correction factor(for mapping motor angle) & kp, ki, kd values respt
	Output: Using calculate function, motor angle 
	"""
	def __init__(self, fact, param, constrains):
		"""
		Initializing PID class variables
		"""
		# rospy.loginfo("PID class Initialized")
		self.kp = param[0]
		self.ki = param[1]
		self.kd = param[2]

		self.time1 = 0.0
		self.time2 = int(round(time.time() * 1000))
		self.total_time = None #(self.time2) - (self.time1)
		
		self.error = 0
		self.prev_error = 0

		self.proportional = 0
		self.integral = 0
		self.derivative = 0

		self.correction = 0
		self.correction_factor = fact
		self.motor_angle = 0

		self.contrain = constrains
		self.smooth_over_values = 50

	def calculate(self, error):
		"""
		Function to calculate PID output
		Input: Error value of the system
		Output: Motor angle for correction
		"""
		# rospy.loginfo("Calculating function called")
		self.error = error
		self.time2 = int(round(time.time() * 1000))
		self.total_time = self.time2 - self.time1
		
		if self.total_time == 0.0:
			self.total_time = 1.0
		else:
			self.total_time = self.total_time

		# print(self.time1, self.time2, self.total_time)
		self.proportional = self.error
		self.integral = (self.error + self.prev_error) * self.total_time
		self.derivative = (self.error - self.prev_error) / self.total_time

		self.correction = (self.kp * self.proportional) + (self.ki * self.integral) + (self.kd * self.derivative)

		self.motor_angle = self. correction * self.correction_factor

		self.time1 = self.time2
		self.prev_error = self.error
		return self.motor_angle

	def smooth(self, motor_angle):
		"""
		Internal function for smoothening output motor angle
		Input: motor angle for various time instances
		Output: smoothed motor angle over number of values
		""" 
		# rospy.loginfo("smooth function called")
		global stable
		self.motor_angle = motor_angle

		if self.motor_angle > self.contrain_straight:   
			self.motor_angle = self.contrain_straight
		elif self.motor_angle < -self.contrain_straight:
			self.motor_angle = -self.contrain_straight

		if len(stable) <= self.smooth_over_values: 
			stable.append(self.motor_angle)
			return None
		else:
			del stable[0]
			stable.append(self.motor_angle)

			self.motor_angle = int(sum(stable) / len(stable))
			return self.motor_angle

	def debug(self):
		"""
		Internal Function for debugging
		"""
		rospy.logdebug("Time-1: ", self.time1, "\t Time-2: ", self.time2, "\t Total time: ", self.total_time)
		rospy.logdebug("Kp: ", self.kp, "\t Ki: ", self.ki, "\t Kd: ", self.kd)
		rospy.logdebug("Error: ", self.error, "\t Prev_error: ", self.prev_error, "\t Motor Angle: ", self.motor_angle)


if __name__ == '__main__':
	test = PID(1, [1, 1, 1])

	test.calculate(1)
