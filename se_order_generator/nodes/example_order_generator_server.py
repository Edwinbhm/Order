#! /usr/bin/env python

#######################################################
# name: order_distributor_server_test.py              #
# author: Thomas A. Theuß V.                          #
# email: uvvmk@student.kit.edu                        #
# description: A test action server for showcasing    #
#              the functionality of 	              #
#              the corresponding action client        #
#######################################################

import rospy
import roslib
import rospkg
import actionlib
import sys
import time
from se_msgs.msg import orderAction, orderResult


###########################################################################################################
# name: OrderActionServer
# type: class
# description: Class which creates a test action server, to be used as an example of how to communicate
#              with the order_distributor_client. 
###########################################################################################################
class OrderActionServer:
    #Creates the result "variable"
    _result = orderResult()

    #-----------------------------------------------------------------------------------------------------#
    # name: __init__
    # type: function
    # description: Function that gets called when creating an object from this class. It creates and starts
    #              the action server
    # param: name - name of the server                  
    #-----------------------------------------------------------------------------------------------------#
    def __init__(self, name):
        self.action_name = name
        self.suffix = "[Test Order Server]"
        rospy.loginfo('%s: Creating server',self.suffix)
        self.action_server = actionlib.ActionServer(self.action_name, orderAction, self.execute_cb, auto_start =False)
        rospy.loginfo('%s: Server created',self.suffix)

        rospy.loginfo('%s: Starting server',self.suffix)
        self.action_server.start()
        rospy.loginfo('%s: Server started',self.suffix)

    #-----------------------------------------------------------------------------------------------------#
    # name: execute_cb
    # type: function
    # description: Callback function of the action server. Here the goal is processed and executed.
    #              If there is a successful completion of the task, the result is sent to the action client
    # param: goal - the goal defined in order_distributor_client.py                 
    #-----------------------------------------------------------------------------------------------------#
    def execute_cb(self,server_goal):
        #Accept goal, thus setting it to state 'ACTIVE'
        server_goal.set_accepted()
        #Define and set goal
        goal = server_goal.get_goal()
        #helper variables
        success = False        
        goal_list = []

        #Set box elements into list (different colours and sizes...)
        for x in range(goal.number_of_boxes):
            goal_list.append(goal.boxes[x])

        #execute action
        for k in range(0, goal.number_of_boxes):
            rospy.loginfo("%s: " + str(goal_list[k]),self.suffix)  #In this case, the action is just printing (you can call your program)
            rospy.sleep(1)                                  #Waits one second (not necessary, just for showcase)
            success = True                                 

        #Time when done
        t = time.localtime(time.time())
        ending_time_seconds = t.tm_sec
        ending_time_minutes = t.tm_min

        if success:
            self._result.ready_flag = success
            #ending time is the time needed to complete the action
            self._result.end_time = str(ending_time_minutes*60 + ending_time_seconds - int(goal.starting_time))
            rospy.loginfo("%s: done\n-----------------",self.suffix)
            server_goal.set_succeeded(self._result)
                  
        
if __name__ == '__main__':
    rospy.init_node('example_order_action_server')
    server = OrderActionServer('order_generator')
    rospy.spin()