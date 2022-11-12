#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import *
import random
import math
import numpy as np
import csv
import rospkg
import time
import os

from erforce.model import RobotCommand, MoveLocalVelocity
from erforce.client import ErForceClientBlue
from erforce.client import ErForceClientYellow
clientblue = ErForceClientBlue()
clientyellow = ErForceClientYellow()

# myCmd = '/home/ricardo/ssl_ws/src/grSim/bin/grSim'
# os.system(myCmd)

def move_robot_blue(data, id):
    global client

    move_command = MoveLocalVelocity(forward=data.linear.x, left=data.linear.y, angular=data.angular.z)
    robot_command = RobotCommand(robot_id=id, move_command=move_command, kick_speed=0, kick_angle=0, dribbler_speed=0)

    clientblue.send_action_command(robot_command)

def move_robot_yellow(data, id):
    global client

    move_command = MoveLocalVelocity(forward=data.linear.x, left=data.linear.y, angular=data.angular.z)
    robot_command = RobotCommand(robot_id=id, move_command=move_command, kick_speed=0, kick_angle=0, dribbler_speed=0)

    clientyellow.send_action_command(robot_command)

def cmd_vel_blue_0_callback(data):
    move_robot_blue(data, 0)

def cmd_vel_blue_1_callback(data):
    move_robot_blue(data, 1)

def cmd_vel_blue_2_callback(data):
    move_robot_blue(data, 2)

def cmd_vel_blue_3_callback(data):
    move_robot_blue(data, 3)

def cmd_vel_blue_4_callback(data):
    move_robot_blue(data, 4)

def cmd_vel_yellow_0_callback(data):
    move_robot_yellow(data, 0)

def cmd_vel_yellow_1_callback(data):
    move_robot_yellow(data, 1)

def cmd_vel_yellow_2_callback(data):
    move_robot_yellow(data, 2)

def cmd_vel_yellow_3_callback(data):
    move_robot_yellow(data, 3)

def cmd_vel_yellow_4_callback(data):
    move_robot_yellow(data, 4)

if __name__ == "__main__":
    rospy.init_node("grsim_ros_bridge", anonymous=False)

    rospy.Subscriber("robot_blue_0/cmd_vel", Twist, cmd_vel_blue_0_callback)
    rospy.Subscriber("robot_blue_1/cmd_vel", Twist, cmd_vel_blue_1_callback)
    rospy.Subscriber("robot_blue_2/cmd_vel", Twist, cmd_vel_blue_2_callback)
    rospy.Subscriber("robot_blue_3/cmd_vel", Twist, cmd_vel_blue_3_callback)
    rospy.Subscriber("robot_blue_4/cmd_vel", Twist, cmd_vel_blue_4_callback)

    rospy.Subscriber("robot_yellow_0/cmd_vel", Twist, cmd_vel_yellow_0_callback)
    rospy.Subscriber("robot_yellow_1/cmd_vel", Twist, cmd_vel_yellow_1_callback)
    rospy.Subscriber("robot_yellow_2/cmd_vel", Twist, cmd_vel_yellow_2_callback)
    rospy.Subscriber("robot_yellow_3/cmd_vel", Twist, cmd_vel_yellow_3_callback)
    rospy.Subscriber("robot_yellow_4/cmd_vel", Twist, cmd_vel_yellow_4_callback)

    rospy.spin()
