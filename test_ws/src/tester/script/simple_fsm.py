#!/usr/bin/env python

import rospy
from smach import State, StateMachine
from time import sleep

class One(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        
    def execute(self, userdata):
        print('one')
        sleep(1)
        return 'success'
    
class Two(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        
    def execute(self, userdata):
        print('two')
        sleep(1)
        return 'success'
    
    
if __name__ == '__main__':
    
    # rospy.init_node('test_fsm', anonymous=True)
    sm = StateMachine(outcomes=['success'])
    
    with sm:
        StateMachine.add('ONE', One(), transitions={'success':'TWO'})
        StateMachine.add('TWO', Two(), transitions={'success':'ONE'})
        
    sm.execute()
    
    
    
    

# class A(State):
#     def __init__(self):
#         State.__init__(self, outcomes=['1','0'], input_keys=['input'],
#         output_keys=[''])
        
#     def execute(self, userdata):
#         sleep(1)
#         if userdata.input == 1:
#             return '1'
#         else:
#             return '0'
        
# rospy.init_node('test_fsm', anonymous=True)
# sm = StateMachine(outcomes=['success'])
# sm.userdata.sm_input = 1

# with sm:
#     StateMachine.add('A', A(), transitions={'1':'B','0':'D'},
#     remapping={'input':'sm_input','output':''})
#     StateMachine.add('B', B(), transitions={'1':'C','0':'A'},
#     remapping={'input':'sm_input','output':''})
#     StateMachine.add('C', C(), transitions={'1':'D','0':'B'},
#     remapping={'input':'sm_input','output':''})
#     StateMachine.add('D', D(), transitions={'1':'A','0':'C'},
#     remapping={'input':'sm_input','output':''})
    
    
# # IntrospectionServer:
#     sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
#     sis.start()    
    
#     sm.execute()
#     rospy.spin()
#     sis.stop()
    
    