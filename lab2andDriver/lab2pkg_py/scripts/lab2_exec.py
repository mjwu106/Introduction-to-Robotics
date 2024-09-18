#!/usr/bin/env python3

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# Hanoi tower location 1
Q11 = [120*pi/180.0, -56*pi/180.0, 124*pi/180.0, -158*pi/180.0, -90*pi/180.0, 0*pi/180.0]
Q12 = [120*pi/180.0, -64*pi/180.0, 123*pi/180.0, -148*pi/180.0, -90*pi/180.0, 0*pi/180.0]
Q13 = [120*pi/180.0, -72*pi/180.0, 120*pi/180.0, -137*pi/180.0, -90*pi/180.0, 0*pi/180.0]

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

start_p = 0
buffer_p = 0
end_p = 0


# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

q11 = [149.12*pi/180.0, -43.96*pi/180.0, 89.62*pi/180.0, -133.72*pi/180.0, -91.97*pi/180.0, 44.41*pi/180.0]
q12 = [149.69*pi/180.0, -49.71*pi/180.0, 89.11*pi/180.0, -127.05*pi/180.0, -91.95*pi/180.0, 44.99*pi/180.0]
q13 = [149.11*pi/180.0, -53.60*pi/180.0, 86.03*pi/180.0, -120.09*pi/180.0, -91.98*pi/180.0, 44.43*pi/180.0]
q21 = [163.57*pi/180.0, -44.82*pi/180.0, 95.01*pi/180.0, -140.20*pi/180.0, -91.22*pi/180.0, 58.53*pi/180.0]
q22 = [163.26*pi/180.0, -51.08*pi/180.0, 94.29*pi/180.0, -130.50*pi/180.0, -91.34*pi/180.0, 58.56*pi/180.0]
q23 = [163.25*pi/180.0, -56.53*pi/180.0, 91.78*pi/180.0, -122.53*pi/180.0, -91.35*pi/180.0, 58.58*pi/180.0]
q31 = [177.36*pi/180.0, -45.98*pi/180.0, 93.83*pi/180.0, -134.92*pi/180.0, -90.64*pi/180.0, 72.62*pi/180.0]
q32 = [177.35*pi/180.0, -51.51*pi/180.0, 92.56*pi/180.0, -128.13*pi/180.0, -90.65*pi/180.0, 72.64*pi/180.0]
q33 = [177.35*pi/180.0, -55.61*pi/180.0, 90.53*pi/180.0, -121.99*pi/180.0, -90.66*pi/180.0, 72.65*pi/180.0]

freeA = [149.11*pi/180.0, -60.49*pi/180.0, 76.83*pi/180.0, -104.01*pi/180.0, -91.99*pi/180.0, 44.47*pi/180.0]
freeB = [163.25*pi/180.0, -62.23*pi/180.0, 86.09*pi/180.0, -111.16*pi/180.0, -91.35*pi/180.0, 58.59*pi/180.0]
freeC = [177.34*pi/180.0, -60.01*pi/180.0, 86.59*pi/180.0, -113.65*pi/180.0, -90.65*pi/180.0, 72.66*pi/180.0]

Q = [ [q11, q12, q13], \
      [q21, q22, q23], \
      [q31, q32, q33] ]

free = [freeA, freeB, freeC]


############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""

def gripper_callback(msg):
    global digital_in_0

    digital_in_0 = msg.DIGIN


############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q
    global free

    move_arm(pub_cmd, loop_rate, free[start_loc], 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, Q[start_loc][start_height], 4.0, 4.0)

    gripper(pub_cmd, loop_rate, 1)
    time.sleep(1.0)   

    if (digital_in_0 == 0): 
        gripper(pub_cmd, loop_rate, 0)
        sys.exit()

    move_arm(pub_cmd, loop_rate, free[start_loc], 4.0, 4.0)

    move_arm(pub_cmd, loop_rate, free[end_loc], 4.0, 4.0)
    move_arm(pub_cmd, loop_rate, Q[end_loc][end_height], 4.0, 4.0)
    gripper(pub_cmd, loop_rate, 0)
    time.sleep(1.0)
    move_arm(pub_cmd, loop_rate, free[end_loc], 4.0, 4.0)







    ### Hint: Use the Q array to map out your towers by location and "height".

    error = 0



    return error


############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    global start_p
    global buffer_p
    global end_p

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function

    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback)
    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    input_done = 0
    #loop_count = 0

    while(not input_done):

        input_string = input("Enter start position <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_string + "\n")

        if (int(input_string) == 1 or int(input_string) == 2 or int(input_string) == 3):
            start_p = int(input_string) - 1
            input_done = 1
        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")

    input_done = 0
    while(not input_done):

        input_string = input("Enter end position <Either 1 2 3 or 0 to quit> ")
        print("You entered " + input_string + "\n")

        if (int(input_string) == start_p+1):
            print("Please give another number")
        elif (int(input_string) == 1 or int(input_string) == 2 or int(input_string) == 3):
            end_p = int(input_string) - 1
            input_done = 1
        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")

    buffer_p = 3 - start_p - end_p

        # if(int(input_string) == 1):
        #     input_done = 1
        #     loop_count = 1
        # elif (int(input_string) == 2):
        #     input_done = 1
        #     loop_count = 2
        # elif (int(input_string) == 3):
        #     input_done = 1
        #     loop_count = 3
        # elif (int(input_string) == 0):
        #     print("Quitting... ")
        #     sys.exit()
        # else:
        #     print("Please just enter the character 1 2 3 or 0 to quit \n\n")





    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input

    # while(loop_count > 0):

    #     move_arm(pub_command, loop_rate, home, 4.0, 4.0)

    #     rospy.loginfo("Sending goal 1 ...")
    #     move_arm(pub_command, loop_rate, Q[0][0], 4.0, 4.0)

    #     gripper(pub_command, loop_rate, suction_on)
    #     # Delay to make sure suction cup has grasped the block
    #     time.sleep(1.0)

    #     rospy.loginfo("Sending goal 2 ...")
    #     move_arm(pub_command, loop_rate, Q[1][1], 4.0, 4.0)

    #     rospy.loginfo("Sending goal 3 ...")
    #     move_arm(pub_command, loop_rate, Q[2][0], 4.0, 4.0)

    #     loop_count = loop_count - 1

    # gripper(pub_command, loop_rate, suction_off)

    move_block(pub_command, loop_rate, start_p,  2, end_p,    0)
    move_block(pub_command, loop_rate, start_p,  1, buffer_p, 0)
    move_block(pub_command, loop_rate, end_p,    0, buffer_p, 1)
    move_block(pub_command, loop_rate, start_p,  0, end_p,    0)
    move_block(pub_command, loop_rate, buffer_p, 1, start_p,  0)
    move_block(pub_command, loop_rate, buffer_p, 0, end_p,    1)
    move_block(pub_command, loop_rate, start_p,  0, end_p,    2)



    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
