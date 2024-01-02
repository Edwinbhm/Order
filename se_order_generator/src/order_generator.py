#!/usr/bin/env python

#######################################################
# name: order_generator.py                            #
# author: Thomas A. Theuß V.                          #
# email: uvvmk@student.kit.edu                        #
# description: Reads/writes order file in json Format #
#######################################################

import sys
import time
import json
import os
import random


###########################################################################################################
# name: OrderGen
# type: class
# description: Class with name "order" which is responsible for generating the order message from a .json
#              file. It can also create "random" .json-files containing the order.
###########################################################################################################
class OrderGen:
    #Variables
    BRB = []    #Large red box
    BBB = []    #Large blue box    
    BGB = []    #Large green box
    SRB = []    #Small red box
    SBB = []    #Small blue box
    SGB = []    #Small green box
    number_boxes = 0
    
    file_name = "example" + ".json"
    file_path = "path"


    #-----------------------------------------------------------------------------------------------------#
    # name: __init__
    # type: function
    # description: Function that gets called when creating object from class "order". Path input is 
    #              transfered to the main function              
    #-----------------------------------------------------------------------------------------------------#
    def __init__(self,order_decision, file_path, file_name, max_num_of_boxes, max_num_orders):
        self.main_function(order_decision, file_path, file_name, max_num_of_boxes, max_num_orders)
    

    #-----------------------------------------------------------------------------------------------------#
    # name: main_function
    # type: function
    # description: Function that gets called by __init__ function. It has the main functionality of the
    #              order generator. It takes the path, where the json files are located and written as an
    #              input.
    #-----------------------------------------------------------------------------------------------------#
    def main_function(self,order_decision, file_path, file_name, max_num_of_boxes, max_num_orders):
        #Find absolute path for json files 
        self.file_path = file_path
        
        self.file_name = self.file_path + "/" + file_name + ".json"

        self.order_decision = order_decision

        self.number_boxes = max_num_of_boxes

        length_dictionary = 0

        #First the program will ask how the order should be created/readed. 
        #print("Order-Generator\n")
        #print("----------------\n")
        #print("For creating order type 'C', for reading order from existing file type 'R'\n")

        if order_decision == 1 or order_decision == 2:
            #print("For custom order type 'custom' and for random type 'random'\n")
            #creation_type = input()
            if order_decision == 1:
                for i in range(max_num_orders):
                    self.create_custom_order(i)
                self.write_to_json(max_num_orders)
            elif order_decision == 2:
                
                for i in range(max_num_orders):
                    self.random_order(self.number_boxes)
                self.write_to_json(max_num_orders)                           
        elif order_decision == 0:
            #print("Please type name of file: ")
            self.file_name = self.file_path + "/" + file_name + ".json"
            
            #print("Path:" + self.file_name)
            self.max_num_order = self.acquire_order()

    #-----------------------------------------------------------------------------------------------------#
    # name: acquire_order
    # type: function
    # description: Function that acquires order information from json-File, which is specified by __init__             
    #-----------------------------------------------------------------------------------------------------#    
    def acquire_order(self):
        done = 0
        data = {}
        #print("Acquiring order from file '" + self.file_name + "'")
        while(done == 0):
            try:
                file_json = open(self.file_name)
                data = json.load(file_json)

                for k in range(len(data)):
                    for i in data[k]:
                        if i == "large_red":
                            self.BRB.append(int(data[k][i]))
                            #self.BRB = int(data[k][i])
                        elif i == "large_blue":
                            self.BBB.append(int(data[k][i]))
                            #self.BBB = int(data[k][i])
                        elif i == "large_green":
                            self.BGB.append(int(data[k][i]))
                            #self.BGB = int(data[k][i])
                        elif i == "small_red":
                            self.SRB.append(int(data[k][i]))
                            #self.SRB = int(data[k][i])
                        elif i == "small_blue":
                            self.SBB.append(int(data[k][i]))
                            #self.SBB = int(data[k][i])
                        elif i == "small_green":
                            self.SGB.append(int(data[k][i]))
                            #self.SGB = int(data[k][i])
                        else:
                            print("Box of type " + i + " not defined: Order might not get processed fully")

                # Closing file
                file_json.close()
                done = 1
                #print('done')
            except:
                #print("File not found. Please try again.")
                done = 1

        return len(data)


    #-----------------------------------------------------------------------------------------------------#
    # name: random_order
    # type: function
    # description: Function that creates a random order             
    #-----------------------------------------------------------------------------------------------------#
    def random_order(self, max_number_of_boxes):
        number_of_boxtypes = 6
        max_nr_box = 3  #tray can carry max 3 boxes at a time
        BRB = 0
        BBB = 0
        BGB = 0
        SRB = 0
        SBB = 0
        SGB = 0

        sum_of_boxes = 0

        while(sum_of_boxes != max_number_of_boxes):
            box_type = random.randint(1,number_of_boxtypes)
            
            if box_type == 1: #large red box
                BRB = int(random.randint(0,max_nr_box))
            elif box_type == 2: #large blue box
                BBB = int(random.randint(0,max_nr_box))
            elif box_type == 3: #large green box
                BGB = int(random.randint(0,max_nr_box))
            elif box_type == 4: #small red box
                SRB = int(random.randint(0,max_nr_box))
            elif box_type == 5: #small blue box
                SBB = int(random.randint(0,max_nr_box))
            elif box_type == 6: #small green box
                SGB = int(random.randint(0,max_nr_box))

            sum_of_boxes = BRB + BBB + BGB + SRB + SBB + SGB

            if sum_of_boxes > max_number_of_boxes:
                if box_type == 1: #large red box
                    #self.BRB.append(int(random.randint(0,max_nr_box)))
                    BRB = int(random.randint(0,max_nr_box))
                elif box_type == 2: #large blue box
                    #self.BBB.append(int(random.randint(0,max_nr_box)))
                    BBB = int(random.randint(0,max_nr_box))
                elif box_type == 3: #large green box
                    #self.BGB.append(int(random.randint(0,max_nr_box)))
                    BGB = int(random.randint(0,max_nr_box))
                elif box_type == 4: #small red box
                    #self.SRB.append(int(random.randint(0,max_nr_box)))
                    SRB = int(random.randint(0,max_nr_box))
                elif box_type == 5: #small blue box
                    #self.SBB.append(int(random.randint(0,max_nr_box)))
                    SBB = int(random.randint(0,max_nr_box))
                elif box_type == 6: #small green box
                    #self.SGB.append(int(random.randint(0,max_nr_box)))
                    SGB = int(random.randint(0,max_nr_box))

        self.BRB.append(BRB)
        self.BBB.append(BBB)
        self.BGB.append(BGB)
        self.SRB.append(SRB)
        self.SBB.append(SBB)
        self.SGB.append(SGB)
        #self.write_to_json()


    #-----------------------------------------------------------------------------------------------------#
    # name: create_custom_order
    # type: function
    # description: Function that creates a custom order, via input             
    #-----------------------------------------------------------------------------------------------------#
    def create_custom_order(self, counter):
        print("Order number: " + str(counter+1) + '\n')
        print("Large red box(es): \n")
        self.BRB.append(input())
        #self.BRB = input()
        print("Large blue box(es): \n")
        self.BBB.append(input())
        #self.BBB = input()
        print("Large green box(es): \n")
        self.BGB.append(input())
        #self.BGB = input()
        print("Small red box(es): \n")
        self.SRB.append(input())
        #self.SRB = input()
        print("Small blue box(es): \n")
        self.SBB.append(input())
        #self.SBB = input()
        print("Small green box(es): \n")
        self.SGB.append(input())
        #self.SGB = input()

        #self.write_to_json(counter)


    #-----------------------------------------------------------------------------------------------------#
    # name: write_to_json
    # type: function
    # description: Function that writes into a json file             
    #-----------------------------------------------------------------------------------------------------#
    def write_to_json(self, max_num_orders):
        # Create list for dictionaries
        orders = []

        # Create dictionaries for json-File
        for k in range(max_num_orders):
            # Initialize lists for boxes
            large_boxes = [("large_red", self.BRB[k]), ("large_blue", self.BBB[k]), ("large_green", self.BGB[k])]
            small_boxes = [("small_red", self.SRB[k]), ("small_blue", self.SBB[k]), ("small_green", self.SGB[k])]

            # Split orders into sub-orders based on box constraints
            while any(box[1] > 0 for box in large_boxes + small_boxes):
                sub_order = {"large_red": 0, "large_blue": 0, "large_green": 0,
                             "small_red": 0, "small_blue": 0, "small_green": 0}

                # Check for 2 small boxes + 1 big box combination
                if sum(box[1] for box in small_boxes) >= 2 and sum(box[1] for box in large_boxes) >= 1:
                    # Add one large box
                    for color, count in large_boxes:
                        if count > 0:
                            sub_order[color] = 1
                            large_boxes[large_boxes.index((color, count))] = (color, count - 1)
                            break

                    # Add two small boxes
                    small_boxes_added = 0
                    for color, count in small_boxes:
                        while count > 0 and small_boxes_added < 2:
                            sub_order[color] += 1
                            count -= 1
                            small_boxes[small_boxes.index((color, count + 1))] = (color, count)
                            small_boxes_added += 1

                # Otherwise, use up to 3 small boxes
                else:
                    small_boxes_added = 0
                    for color, count in small_boxes:
                        while count > 0 and small_boxes_added < 3:
                            sub_order[color] += 1
                            count -= 1
                            small_boxes[small_boxes.index((color, count + 1))] = (color, count)
                            small_boxes_added += 1

                orders.append(sub_order)

        # Serializing json
        json_object = json.dumps(orders, indent=4)

        # Writing to file_name.json
        with open(self.file_name, "w") as outfile:
            outfile.write(json_object)
