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

from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger, TriggerRequest

thetas = [0, 0, 0, 0, 0, 0]
delta_thetas = [0, 0, 0, 0, 0, 0]
button_pressed = False

delta_x = 0
delta_y = 0
delta_z = 0
delta_ang_x = 0
delta_ang_y = 0
delta_ang_z = 0
scaling_factor_linear = 0.005
scaling_factor_angular = 0.1
button_1 = 0
button_2 = 0
button_1_last = 0
button_2_last = 0
pulling_down_l2 = 0
pulling_down_r2 = 0
def joy_callback(data):
	# print(data.axes)
	global pulling_down_l2, pulling_down_r2, delta_x, delta_y, delta_z, delta_ang_x, delta_ang_y, delta_ang_z, button_1, button_2, button_1_last, button_2_last
	delta_x = data.axes[4] * data.buttons[0]
	delta_y = -data.axes[4] * data.buttons[1]
	delta_z = data.axes[4] * data.buttons[2]


	delta_ang_x = data.axes[7] * 0 
	delta_ang_y = -data.axes[6] * 0
	delta_ang_z = data.buttons[3] * 0

	button_1_last = button_1 * 0
	button_2_last = button_2 * 0
	button_1 = data.buttons[1] * 0
	button_2 = data.buttons[2] * 0
 
	pulling_down_l2 = data.axes[2]
	pulling_down_r2 = data.axes[5]

	# print(data.buttons)

rospy.init_node("keyboard_op")

publisher = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size = 10)

joy = rospy.Subscriber('/joy', Joy, joy_callback)

rate = rospy.Rate(80)

rospy.wait_for_service('/servo_server/reset_target')

reset_service = rospy.ServiceProxy('/servo_server/reset_target', Trigger)

while not rospy.is_shutdown():
	# print(delta_x, delta_y, delta_z)
	if (button_1 and not button_1_last):
		scaling_factor_linear += 0.001
	if (button_2 and not button_2_last):
		scaling_factor_linear -= 0.001
	msg = TwistStamped()
	msg.header = Header()
	msg.header.stamp = rospy.Time.now()
	msg.twist.linear.x = delta_x * scaling_factor_linear
	msg.twist.linear.y = delta_y * scaling_factor_linear
	msg.twist.linear.z = delta_z * scaling_factor_linear
	msg.twist.angular.x = delta_ang_x * scaling_factor_linear
	msg.twist.angular.y = delta_ang_y * scaling_factor_linear
	msg.twist.angular.z = delta_ang_z * scaling_factor_linear
 
	if (pulling_down_l2 + pulling_down_r2 < -1.5):
		try:
			response = reset_service()
			print(response.message)
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)
     
	rate.sleep()

	print(msg)
	publisher.publish(msg)
