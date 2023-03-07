#!/usr/bin/env python3
import rospy
import math
import numpy as np
import os
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from grsim_ros_bridge_msgs.msg import SSL
from erforce.model import RobotCommand, MoveLocalVelocity
from erforce.client import ErForceClientBlue, ErForceClientYellow

KICKER_SPEED = 1.0
DIBBLER_SPEED = 1.0

def move_robot(client, data, id):
    move_command = MoveLocalVelocity(forward=data.linear.x, left=data.linear.y, angular=data.angular.z)

    if data.kicker and data.dribbler:
        robot_command = RobotCommand(robot_id=id, move_command=move_command, kick_speed=KICKER_SPEED, kick_angle=0, dribbler_speed=DIBBLER_SPEED)
    elif not data.kicker and data.dribbler:
        robot_command = RobotCommand(robot_id=id, move_command=move_command, kick_speed=0, kick_angle=0, dribbler_speed=DIBBLER_SPEED)
    elif data.kicker and not data.dribbler:
        robot_command = RobotCommand(robot_id=id, move_command=move_command, kick_speed=KICKER_SPEED, kick_angle=0, dribbler_speed=0)
    else:
        robot_command = RobotCommand(robot_id=id, move_command=move_command, kick_speed=0, kick_angle=0, dribbler_speed=0)

    client.send_action_command(robot_command)

def cmd_callback(client, data, id):
    move_robot(client, data, id)

if __name__ == '__main__':
    rospy.init_node('grsim_ros_bridge', anonymous=True)

    blue_clients = [ErForceClientBlue() for _ in range(5)]
    yellow_clients = [ErForceClientYellow() for _ in range(5)]

    blue_cmd_topics = [f'robot_blue_{i}/cmd' for i in range(5)]
    yellow_cmd_topics = [f'robot_yellow_{i}/cmd' for i in range(5)]

    for i in range(5):
        rospy.Subscriber(blue_cmd_topics[i], Twist, cmd_callback, callback_args=(blue_clients[i], i))
        rospy.Subscriber(yellow_cmd_topics[i], Twist, cmd_callback, callback_args=(yellow_clients[i], i))

    rospy.spin()
