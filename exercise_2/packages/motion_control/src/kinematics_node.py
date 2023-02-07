#!/usr/bin/env python3

import rospy
import json
import yaml
import time
import os.path

from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped, Twist2DStamped
from std_srvs.srv import EmptyResponse, Empty

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType


class KinematicsNode(DTROS):
    """
    The `KinematicsNode` maps car commands sent from various nodes to wheel commands
    that the robot can execute.

    The `KinematicsNode` performs both the inverse and forward kinematics calculations.
    
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Subscriber:
        ~car_cmd (:obj:`Twist2DStamped`): The requested car command

    Publisher:
        ~wheels_cmd (:obj:`WheelsCmdStamped`): The corresponding resulting wheel commands
        ~velocity (:obj:`Twist2DStamped`): The open-loop estimation of the robot velocity

    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(KinematicsNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # Get the vehicle name
        self.veh_name = rospy.get_namespace().strip("/")

        # Read parameters from a robot-specific yaml file if such exists
        self.read_params_from_calibration_file()

        # Get static parameters
        self._radius = DTParam("~radius", param_type=ParamType.FLOAT, min_value=0.01, max_value=0.1)

        # -- Setup publishers -- 
        self.pub_wheels_cmd = rospy.Publisher(
            "~wheels_cmd", WheelsCmdStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )
        self.pub_velocity = rospy.Publisher(
            "~velocity", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        # -- Setup subscribers -- 
        self.sub_car_cmd = rospy.Subscriber(
            "~car_cmd", Twist2DStamped, self.car_cmd_callback
        )
        self.sub_encoder_ticks_left = rospy.Subscriber(
            "~left_wheel_encoder_node/tick", WheelEncoderStamped, 
            lambda x: self.cb_encoder_data('left', x)
        )
        self.sub_encoder_ticks_right = rospy.Subscriber(
            "~left_wheel_encoder_node/tick", WheelEncoderStamped, 
            lambda x: self.cb_encoder_data('right', x)
        )
        
        # ---
        self.log("kachow!")


    def car_cmd_callback(self, msg_car_cmd):
        """
        A callback that reposponds to received `car_cmd` messages by calculating the
        corresponding wheel commands, taking into account the robot geometry, gain and trim
        factors, and the set limits. These wheel commands are then published for the motors to use.
        The resulting linear and angular velocities are also calculated and published.

        Args:
            msg_car_cmd (:obj:`Twist2DStamped`): desired car command
        """

        # INVERSE KINEMATICS PART

        omega_r = (msg_car_cmd.v + 0.5 * msg_car_cmd.omega * self._baseline.value) / self._radius.value
        omega_l = (msg_car_cmd.v - 0.5 * msg_car_cmd.omega * self._baseline.value) / self._radius.value

        # conversion from motor rotation rate to duty cycle
        # u_r = (gain + trim) (v + 0.5 * omega * b) / (r * k_r)
        u_r = omega_r * k_r_inv
        # u_l = (gain - trim) (v - 0.5 * omega * b) / (r * k_l)
        u_l = omega_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = self.trim(u_r, -self._limit.value, self._limit.value)
        u_l_limited = self.trim(u_l, -self._limit.value, self._limit.value)

        # Put the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp = msg_car_cmd.header.stamp
        msg_wheels_cmd.vel_right = u_r_limited
        msg_wheels_cmd.vel_left = u_l_limited
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

        # FORWARD KINEMATICS PART

        # Conversion from motor duty to motor rotation rate
        omega_r = msg_wheels_cmd.vel_right / k_r_inv
        omega_l = msg_wheels_cmd.vel_left / k_l_inv

        # Compute linear and angular velocity of the platform
        v = (self._radius.value * omega_r + self._radius.value * omega_l) / 2.0
        omega = (self._radius.value * omega_r - self._radius.value * omega_l) / self._baseline.value

        # Put the v and omega into a velocity message and publish
        msg_velocity = Twist2DStamped()
        msg_velocity.header = msg_wheels_cmd.header
        msg_velocity.v = v
        msg_velocity.omega = omega
        self.pub_velocity.publish(msg_velocity)


if __name__ == "__main__":
    # Initialize the node
    kinematics_node = KinematicsNode(node_name="kinematics_node")
    # Keep it spinning to keep the node alive
    rospy.spin()