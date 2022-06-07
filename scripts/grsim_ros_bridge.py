#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import *
import random
import math
import numpy as np
import csv
import rospkg
import time

from erforce.model import RobotCommand, MoveLocalVelocity
from erforce.client import ErForceClient
client = ErForceClient()

def cmd_vel_callback(data):
    global client

    move_command = MoveLocalVelocity(forward=data.linear.x, left=data.linear.y, angular=data.angular.z)
    robot_command = RobotCommand(robot_id=0, move_command=move_command, kick_speed=0, kick_angle=0, dribbler_speed=0)

    client.send_action_command(robot_command)

if __name__ == "__main__":
    rospy.init_node("grsim_ros_bridge", anonymous=False)

    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

    rospy.spin()
