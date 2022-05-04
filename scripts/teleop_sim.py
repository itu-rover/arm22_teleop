#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Author: 	B. Burak Payzun
# Date: 	2021-08-27
#
###########################


import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import serial

from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np


def deadband(val, deadband):
    if(abs(val) < deadband):
        return 0
    else:
        return val

class TeleopClass:
    
    def js_callback(self, data):
        self.thetas = np.asarray(data.position)
    def loop(self): 
        while not rospy.is_shutdown():
            rospy.loginfo("Last mode was: {} | Current mode is: {}".format(self.LAST_STATE, self.CURRENT_STATE))
            self.rate.sleep()
            
    def undefined_func(self):
        pass
    
    def joint_func(self):        
        is_resetting = (self.data.axes[2] + self.data.axes[5] < -1.5)
        
        self.thetas[0] += (self.data.axes[4] * self.data.buttons[0]) * 0.01
        self.thetas[1] += (self.data.axes[4] * self.data.buttons[1]) * 0.01
        self.thetas[2] += (self.data.axes[4] * self.data.buttons[2]) * 0.01
        self.thetas[3] += (self.data.axes[4] * self.data.buttons[3]) * 0.01
        self.thetas[4] += (self.data.axes[4] * self.data.buttons[4]) * 0.01
        self.thetas[5] += (self.data.axes[4] * self.data.buttons[5]) * 0.01
        msg = JointTrajectory()
        msg.joint_names = ["axis_1", "axis_2", "axis_3", "axis_4", "axis_5", "axis_6"]
        pt = JointTrajectoryPoint()
        pt.positions = self.thetas
        pt.time_from_start = rospy.Duration.from_sec(0.005)
        msg.points = [pt]
        self.arm_publisher.publish(msg)
        
        self.comm_check_byte ^= 1
        message_to_send = ""
        message_to_send += "S"
        message_to_send += str(int(max(0, min(5 +
                                (self.data.axes[4] * self.data.buttons[0])*5, 9))))
        message_to_send += str(int(max(0, min(5 +
                                (-self.data.axes[4] * self.data.buttons[1])*5, 9))))
        message_to_send += str(int(max(0, min(5 +
                                (self.data.axes[4] * self.data.buttons[2])*5, 9))))
        message_to_send += str(int(max(0, min(5 +
                                (self.data.axes[4] * self.data.buttons[3])*5, 9))))
        message_to_send += str(int(max(0, min(5 +
                                (self.data.axes[4] * self.data.buttons[4])*5, 9))))
        message_to_send += str(int(max(0, min(5 +
                                (-self.data.axes[4] * self.data.buttons[5])*5, 9))))
        message_to_send += str(int(max(0, min(5 +
                                (-self.data.axes[4] * self.data.buttons[6])*5, 9))))
        message_to_send += str(self.comm_check_byte)
        message_to_send += "F"
        if is_resetting:
            message_to_send = "RRRRR"
        print(message_to_send)
    
    def cartesian_func(self):
        is_resetting = (self.data.axes[2] + self.data.axes[5] < -1.5)
                
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        speed_modifier = (2 * self.data.buttons[4]) + (2 * self.data.buttons[5]) + 1 # 1, 3, 5
        msg.twist.linear.x =  deadband(self.data.axes[3], 0.15) * self.k_linear * (1-self.data.buttons[0]) * speed_modifier
        msg.twist.linear.y = deadband(-self.data.axes[4], 0.15) * self.k_linear * (1-self.data.buttons[0]) * speed_modifier
        msg.twist.linear.z = deadband(self.data.axes[1], 0.15) * self.k_linear * (1-self.data.buttons[0]) * speed_modifier
        msg.twist.angular.x = deadband(-self.data.axes[1], 0.15) * self.k_linear * self.data.buttons[0] * 5 * speed_modifier
        msg.twist.angular.y = deadband(-self.data.axes[4], 0.15) * self.k_linear * self.data.buttons[0] * 5 * speed_modifier
        msg.twist.angular.z = deadband(self.data.axes[0], 0.15) * self.k_linear * self.data.buttons[0] * 5 * speed_modifier

        if (is_resetting):
            try:
                response = self.reset_service()
                print(response.message)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        print(msg)
        self.publisher.publish(msg)
    
    def joy_callback(self, data):
        self.last_data = self.data
        self.data = data
        
        if (self.data.axes[6] == 1) and self.data.buttons[3] and self.CURRENT_STATE != "STEERING": # STEERING
            self.LAST_STATE = self.CURRENT_STATE
            self.CURRENT_STATE = "STEERING"
            self.allow_service(False)
        if (self.data.axes[7] == 1) and self.data.buttons[3] and self.CURRENT_STATE != "JOINT": # JOINT
            self.LAST_STATE = self.CURRENT_STATE
            self.CURRENT_STATE = "JOINT"
            self.allow_service(False)
        elif (self.data.axes[6] == -1) and self.data.buttons[3] and self.CURRENT_STATE != "CARTESIAN": # CARTESIAN
            if self.service_not_connected:
                rospy.loginfo("Waiting for servo service")
                rospy.wait_for_service('/servo_server/reset_target')
                self.reset_service = rospy.ServiceProxy('/servo_server/reset_target', Trigger)
                self.service_not_connected = False
                rospy.sleep(2)
            if (self.CURRENT_STATE == "JOINT"):
                self.reset_service()
                print("reset target")
                rospy.sleep(2)
            self.LAST_STATE = self.CURRENT_STATE
            self.CURRENT_STATE = "CARTESIAN"
            self.allow_service(True)
            
        self.mode_publisher.publish(String(self.CURRENT_STATE))
        
        self.state_dict[self.CURRENT_STATE]()
        
    def __init__(self):
        self.CURRENT_STATE = "UNDEFINED" # "CARTESIAN" "JOINT"
        self.LAST_STATE = "UNDEFINED" # "CARTESIAN" "JOINT"

        self.k_linear = 0.001
        self.k_angular = 0.1
        
        self.thetas = [0, 0, 0, 0, 0, 0]
        
        self.data = Joy()
        self.data.axes = [0, 0, 0, 0, 0, 0, 0, 0]
        self.data.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.last_data = self.data

        self.state_dict = {"UNDEFINED": self.undefined_func,
                    "CARTESIAN": self.cartesian_func,
                    "JOINT": self.joint_func}
        
        rospy.init_node("teleop_node")

        self.publisher = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size = 10)
        self.arm_publisher = rospy.Publisher('/manipulator_controller/command', JointTrajectory, queue_size=10) #
        self.mode_publisher = rospy.Publisher('/manipulator/current_mode', String, queue_size = 10)
        joy = rospy.Subscriber('/joy', Joy, self.joy_callback)
        joint_states = rospy.Subscriber('/joint_states', JointState, self.js_callback)

        self.service_not_connected = True
        self.rate = rospy.Rate(1)
        self.comm_check_byte = 0

        print("Waiting for toggle service")
        rospy.wait_for_service('/hardware_interface/toggle_write')
        self.allow_service = rospy.ServiceProxy('/hardware_interface/allow_write', SetBool)
        self.reset_service = rospy.ServiceProxy('/servo_server/reset_target', Trigger)

        self.loop()
        
if __name__ == "__main__":
    TeleopClass()