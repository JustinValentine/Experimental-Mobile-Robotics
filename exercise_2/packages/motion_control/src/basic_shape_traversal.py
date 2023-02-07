#!/usr/bin/env python3

import rospy
import json
import yaml
import time
import os.path

from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from std_srvs.srv import EmptyResponse, Empty

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType

class Basic_Shape_Traversal(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(Basic_Shape_Traversal, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

    def turn_in_place(self, theta):
        pass

    def drive_straight(self, distance, speed):
        pass

    def drive_circle(self, radius, speed):
        pass