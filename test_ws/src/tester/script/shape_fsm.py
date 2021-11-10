#! /usr/bin/env python
import rospy
from smach import State, StateMachine
import time

class Drive(State):                               #define the class for this state by inheriting the 'State' class
    def __init__(self, distance):                 #initialize the constructor for this state
        State.__init__(self, outcomes=['success'])   #initialize the constructor of State parent class with all possible outcomes
        self.distance = distance                     #assign the distance data to this class Data member
        
    def execute(self, userdata):                  #body of this state
        print('Driving', self.distance)
        time.sleep(1)
        return 'success'                          # return outcomes which is defined in constructor
    
    
class Turn(State):
    def __init__(self, angle):
        State.__init__(self, outcomes=['success'])
        self.angle = angle
        
    def execute(self, userdata):
        print("Turning", self.angle)
        time.sleep(1)
        return 'success'
    

if __name__ == '__main__':
    triangle = StateMachine(outcomes=['success'])      #creating instance for this statemachine using 'StateMachine()' class
    
    with triangle:                                     #define the hirarchy for this statemachine using triangle instance
        StateMachine.add('SIDE1', Drive(1), transitions={'success':'TURN1'})       #add function with defining state_name('SIDE1'), state_execution body(Drive()) and transition path(if retrun True go to next state)....This is calling as containers to store flow info
        StateMachine.add('TURN1', Turn(120), transitions={'success':'SIDE2'})
        StateMachine.add('SIDE2', Drive(1), transitions={'success':'TURN2'})
        StateMachine.add('TURN2', Turn(120), transitions={'success':'SIDE3'})
        StateMachine.add('SIDE3', Drive(1), transitions={'success':'success'})
        
    
    square = StateMachine(outcomes=['success']) 
    with square:
        StateMachine.add('SIDE1', Drive(1), transitions={'success':'TURN1'})
        StateMachine.add('TURN1', Turn(120), transitions={'success':'SIDE2'})
        StateMachine.add('SIDE2', Drive(1), transitions={'success':'TURN2'})
        StateMachine.add('TURN2', Turn(120), transitions={'success':'SIDE3'})
        StateMachine.add('SIDE3', Drive(1), transitions={'success':'TURN4'})
        StateMachine.add('TURN4', Turn(120), transitions={'success':'SIDE4'})
        StateMachine.add('SIDE4', Drive(1), transitions={'success':'success'})
        
        
    shapes = StateMachine(outcomes=['success'])
    with shapes:                                                                 #Define 2 statemachine hirarchy to chain together using smach
        StateMachine.add('TRIANGLE', triangle, transitions={'success':'SQUARE'})
        StateMachine.add('SQUARE', square, transitions={'success':'success'})
        
    shapes.execute()
        
    
    
    