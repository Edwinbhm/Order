#!/usr/bin/env python

#######################################################
# name: order_distributor_client.py                   #
# author: Thomas A. Theuß V.                          #
# email: uvvmk@student.kit.edu                        #
# description: Creates the interface from json to ros #
#######################################################

import rospy
import roslib
import rospkg
import actionlib
import sys
import time
import datetime
import random

from se_msgs.msg import orderAction, orderGoal, CargoBox

rospack = rospkg.RosPack()
path = rospack.get_path('se_order_generator')
sys.path.append(path)
#tells sys where to look for class
from src.order_generator import OrderGen

#Get parameters from launch files
#order_decision = rospy.get_param("/order_decision")
if rospy.has_param('/file_path'):
   #Read
   order_decision = 0
   file_path = rospy.get_param("/file_path")
   file_name = rospy.get_param("/file_name")
   max_num_of_boxes = 0
   max_num_orders = 0
elif rospy.has_param('/max_num_orders') and not rospy.has_param('/max_num_of_boxes'):
   #custom
   order_decision = 1
   file_path = path + '/json'    #default file path
   file_name = 'customOrder'
   max_num_of_boxes = 0
   max_num_orders = rospy.get_param("/max_num_orders")
else:
   #random
   order_decision = 2
   file_path = path + '/json'    #default file path
   file_name = 'randomOrder'
   max_num_orders = rospy.get_param("/max_num_orders")
   max_num_of_boxes = rospy.get_param("/max_num_of_boxes")
      

max_time = rospy.get_param("/max_time")
max_time_minutes = rospy.get_param("/max_time_minutes")
max_time_seconds = rospy.get_param("/max_time_seconds")
heigth_large_boxes = float(rospy.get_param("/height_large_boxes"))
width_large_boxes = float(rospy.get_param("/width_large_boxes"))
length_large_boxes = float(rospy.get_param("/length_large_boxes"))
height_small_boxes = float(rospy.get_param("/height_small_boxes"))
width_small_boxes = float(rospy.get_param("/width_small_boxes"))
length_small_boxes = float(rospy.get_param("/length_small_boxes"))
suffix = "[Order Action Client]"

#-----------------------------------------------------------------------------------------------------#
# name: order_client
# type: function
# description: Function that creates the action client for the order generator
# param: order_number - current order number
#        number_of_boxes - number of boxes per order
#        large/small_red/blue/green_boxes - number of coloured boxes of the different sizes                  
#-----------------------------------------------------------------------------------------------------#
def order_client(order_number,number_of_boxes,large_red_boxes,large_blue_boxes,large_green_boxes,
                 small_red_boxes,small_blue_boxes,small_green_boxes):
   #Set success flag to False by default
   success = False

   #Create the action client
   rospy.loginfo('%s: Order Nr.: ' + str(order_number),suffix)
   rospy.loginfo('%s: Creating client',suffix)
   client = actionlib.ActionClient('order_generator', orderAction)
   rospy.loginfo('%s: Client creation completed',suffix)

   #Waits for response of action server
   rospy.loginfo('%s: Waiting for server',suffix)
   client.wait_for_server()
   rospy.loginfo('%s: Connected to server',suffix) 

   #Define the message type
   cargoBox = []

   #loop through number of boxes and save in list
   if(large_red_boxes != 0):
      for x in range(large_red_boxes):
         cargoBox.append(assign_to_msg(large_red_boxes,"01","red",heigth_large_boxes,width_large_boxes,length_large_boxes))

   if(large_blue_boxes != 0):
      for x in range(large_blue_boxes):
         cargoBox.append(assign_to_msg(large_blue_boxes,"02","blue",heigth_large_boxes,width_large_boxes,length_large_boxes))

   if(large_green_boxes != 0):
      for x in range(large_green_boxes):
         cargoBox.append(assign_to_msg(large_green_boxes,"03","green",heigth_large_boxes,width_large_boxes,length_large_boxes))


   if(small_red_boxes != 0):
      for x in range(small_red_boxes):
         cargoBox.append(assign_to_msg(small_red_boxes,"04","red",height_small_boxes,width_small_boxes,length_small_boxes))

   if(small_blue_boxes != 0):
      for x in range(small_blue_boxes):
         cargoBox.append(assign_to_msg(small_blue_boxes,"05","blue",height_small_boxes,width_small_boxes,length_small_boxes))

   if(small_green_boxes != 0):
      for x in range(small_green_boxes):
         cargoBox.append(assign_to_msg(small_green_boxes,"06","green",height_small_boxes,width_small_boxes,length_small_boxes))
   
   #Defines the goal
   rospy.loginfo('%s: Defining goal',suffix)  
   goal = orderGoal()
   goal.order_number = order_number
   goal.number_of_boxes = number_of_boxes
   
   for x in range(goal.number_of_boxes):
      goal.boxes.append(cargoBox[x])

   t = rospy.get_time()
   starting_time = t

   t = time.localtime(time.time())
   begin = time.asctime(t)
   starting_time_minutes = t.tm_min
   starting_time_seconds = t.tm_sec

   goal.starting_time = str(starting_time_minutes*60 + starting_time_seconds)

   rospy.loginfo('%s: Starting time: ' + goal.starting_time,suffix)
   
   #Sends the goal to the server
   rospy.loginfo('%s: Sending goal',suffix)
   clientHandler = None
   clientHandler = client.send_goal(goal)
   rospy.loginfo('%s: Goal sent to server',suffix)

   rospy.loginfo('%s: Waiting for result',suffix) 

   #Creates the total number of seconds for the counter
   total_seconds = max_time_minutes*60 + max_time_seconds

   #While loop that checks if total_seconds reaches zero
   #If not zero, decrement total time by one second
   while total_seconds > 0:
      #Delays the program one second
      rospy.sleep(1)

      #Reduces total time by one second
      total_seconds -= 1
      
      #If the server is finished, then the counter gets cancelled
      if clientHandler.get_goal_status() == actionlib.GoalStatus.SUCCEEDED:
         #rospy.loginfo(str(actionlib.SimpleGoalState.DONE))
         success = True
         t = rospy.get_time()
         ending_time = t
         result = ending_time - starting_time
         rospy.loginfo('%s: Got result',suffix)
         rospy.loginfo('%s: Result: ' + '\n' + str(clientHandler.get_result()),suffix)
         rospy.loginfo('%s: ----------------------',suffix)
         return client.ActionResult
   
   #Cancels the goal
   client.cancel_all_goals()
   rospy.loginfo('%s: Maximum allowed time of %i minutes and %i seconds for order reached.',suffix,
                  max_time_minutes, max_time_seconds)
   rospy.loginfo('%s: ----------------------',suffix)
   return False

#-----------------------------------------------------------------------------------------------------#
# name: assign_to_msg
# type: function
# description: Function which saves box parameters into a list         
#-----------------------------------------------------------------------------------------------------#
def assign_to_msg(number_of_color_boxes, box_id, box_color, heigth, width, length):
   new_cargoBox = CargoBox()
   
   new_cargoBox.id = box_id
   new_cargoBox.color = box_color
   new_cargoBox.heigth = heigth
   new_cargoBox.width = width
   new_cargoBox.length = length

   return new_cargoBox

#-----------------------------------------------------------------------------------------------------#
# name: main_client
# type: function
# description: Function with the main functionality of the client. Creates a node and calls the 
#              order_client function           
#-----------------------------------------------------------------------------------------------------#
def main_client(args):
   #Creates the node
   rospy.init_node('order_action_client')

   #Creates the object from the order_gen class 
   Order = OrderGen(order_decision, file_path, file_name, max_num_of_boxes, max_num_orders)

   #Decides if a json file is read or written
   if Order.order_decision == 1 or Order.order_decision == 2:
      #custom or random order creation
      rospy.loginfo('%s: Creating file:' +  str(Order.file_name), suffix)
   else:
      #Acquiring json file
      rospy.loginfo('%s: ----------------------',suffix)
      for i in range(Order.max_num_order):
         boxes = Order.BRB[i] + Order.BBB[i] + Order.BGB[i] + Order.SRB[i] + Order.SBB[i] + Order.SGB[i]
         result = order_client(i+1,boxes, Order.BRB[i], Order.BBB[i], Order.BGB[i], Order.SRB[i], Order.SBB[i], Order.SGB[i])


if __name__ == '__main__':
    main_client(sys.argv)
