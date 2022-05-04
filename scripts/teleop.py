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

silent = False


def deadband(val, deadband):
    if(abs(val) < deadband):
        return 0
    else:
        return val


class TeleopClass:
    def __init__(self):
        self.k_linear = 0.001
        self.k_angular = 0.1

        # one of "UNDEFINED" "STEERING" "CARTESIAN" "JOINT" "POSE"
        self.CURRENT_STATE = "UNDEFINED"
        self.LAST_STATE = "UNDEFINED"

        # TO-DO: use the same parameters as arm22_control
        self.ser = serial.Serial("/dev/roverHeroSteering", 115200)

        self.data = Joy()
        self.data.axes = [0, 0, 0, 0, 0, 0, 0, 0]
        self.data.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.last_data = self.data

        self.comm_check_byte = 0
        self.service_not_connected = True

        self.state_dict = {"UNDEFINED": self.undefined_func,
                           "STEERING": self.undefined_func,
                           "SWITCHING_TO_CARTESIAN": self.undefined_func,
                           "POSE": self.pose_func,
                           "CARTESIAN": self.cartesian_func,
                           "JOINT": self.joint_func}

        rospy.init_node("teleop_node")

        self.publisher = rospy.Publisher(
            '/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)
        self.arm_publisher = rospy.Publisher(
            '/manipulator_controller/command', JointTrajectory, queue_size=10)
        self.mode_publisher = rospy.Publisher(
            '/manipulator/current_mode', String, queue_size=10)
        self.status_publisher = rospy.Publisher(
            '/manipulator/status', String, queue_size=10)
        joy = rospy.Subscriber('/joy', Joy, self.joy_callback)

        self.rate = rospy.Rate(1)

        if not silent:
            print("Waiting for toggle service")
        rospy.wait_for_service('/hardware_interface/allow_write')
        self.allow_service = rospy.ServiceProxy(
            '/hardware_interface/allow_write', SetBool)
        self.reset_service = rospy.ServiceProxy(
            '/servo_server/reset_target', Trigger)

        self.loop()

    def loop(self):
        while not rospy.is_shutdown():
            rospy.loginfo("Last mode was: {} | Current mode is: {}".format(
                self.LAST_STATE, self.CURRENT_STATE))
            self.rate.sleep()

    def undefined_func(self):
        pass

    def pose_func(self):
        status = ""
        self.status_publisher.publish(status)
        self.allow_service(True)

    def joint_func(self):
        is_resetting = (self.data.axes[2] + self.data.axes[5] < -1.5)

        if(self.LAST_STATE != "JOINT"):
            self.allow_service(False)

        self.comm_check_byte ^= 1
        message_to_send = "S"
        message_to_send += str(int(max(0, min(5 +
                                              (self.data.axes[4] * self.data.buttons[0])*5, 9))))
        message_to_send += str(int(max(0, min(5 +
                                              (self.data.axes[4] * self.data.buttons[1])*5, 9))))
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

        if not silent:
            print(message_to_send)

        self.status_publisher.publish(message_to_send)
        self.ser.write(message_to_send)

    def cartesian_func(self):
        is_resetting = (self.data.axes[2] + self.data.axes[5] < -1.5)

        if(self.LAST_STATE != "CARTESIAN"):
            self.allow_service(True)

        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        speed_modifier = (
            2 * self.data.buttons[4]) + (2 * self.data.buttons[5]) + 1  # 1, 3, 5
        msg.twist.linear.x = deadband(
            self.data.axes[3], 0.15) * self.k_linear * (1-self.data.buttons[0]) * speed_modifier
        msg.twist.linear.y = deadband(-self.data.axes[4], 0.15) * self.k_linear * (
            1-self.data.buttons[0]) * speed_modifier
        msg.twist.linear.z = deadband(
            self.data.axes[1], 0.15) * self.k_linear * (1-self.data.buttons[0]) * speed_modifier
        msg.twist.angular.x = deadband(-self.data.axes[1], 0.15) * \
            self.k_linear * self.data.buttons[0] * 5 * speed_modifier
        msg.twist.angular.y = deadband(-self.data.axes[4], 0.15) * \
            self.k_linear * self.data.buttons[0] * 5 * speed_modifier
        msg.twist.angular.z = deadband(
            self.data.axes[0], 0.15) * self.k_linear * self.data.buttons[0] * 5 * speed_modifier

        if (is_resetting):
            try:
                response = self.reset_service()
                print(response.message)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

        status = "L x: {:.3f} y: {:.3f} z: {:.3f} \nA x: {:.3f} y: {:.3f} z: {:.3f}".format(msg.twist.linear.x*100, msg.twist.linear.y*100, msg.twist.linear.z*100,
                                                                                            msg.twist.angular.x*100, msg.twist.angular.y*100, msg.twist.angular.z*100)
        if not silent:
            print(status)
        self.status_publisher.publish(status)
        self.publisher.publish(msg)

    def update_state(self, data):
        if (self.data.axes[6] == 1) and self.data.buttons[6] and self.CURRENT_STATE != "STEERING":  # STEERING
            self.LAST_STATE = self.CURRENT_STATE
            self.CURRENT_STATE = "STEERING"
        if (self.data.axes[7] == 1) and self.data.buttons[6] and self.CURRENT_STATE != "JOINT":  # JOINT
            self.LAST_STATE = self.CURRENT_STATE
            self.CURRENT_STATE = "JOINT"
        if (self.data.axes[7] == -1) and self.data.buttons[6] and self.CURRENT_STATE != "POSE":  # POSE
            self.LAST_STATE = self.CURRENT_STATE
            self.CURRENT_STATE = "POSE"
        elif (self.data.axes[6] == -1) and self.data.buttons[6] and self.CURRENT_STATE != "CARTESIAN":  # CARTESIAN
            self.CURRENT_STATE = "SWITCHING_TO_CARTESIAN"
            if self.service_not_connected:
                if not silent:
                    rospy.loginfo("Waiting for servo service")
                self.status_publisher.publish("Waiting for servo service")
                rospy.wait_for_service('/servo_server/reset_target')
                self.reset_service = rospy.ServiceProxy(
                    '/servo_server/reset_target', Trigger)
                self.service_not_connected = False
                rospy.sleep(2)

            self.reset_service()

            if not silent:
                print("reset target")
            self.status_publisher.publish("Target reset")
            rospy.sleep(2)
            self.LAST_STATE = self.CURRENT_STATE
            self.CURRENT_STATE = "CARTESIAN"

    def joy_callback(self, data):
        self.last_data = self.data
        self.data = data

        self.update_state(data)

        self.mode_publisher.publish(String(self.CURRENT_STATE))

        self.state_dict[self.CURRENT_STATE]()


if __name__ == "__main__":
    TeleopClass()
