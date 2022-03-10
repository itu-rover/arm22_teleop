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
from std_srvs.srv import Trigger, TriggerRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def deadband(val, deadband):
    if(abs(val) < deadband):
        return 0
    else:
        return val

CURRENT_MODE = "UNDEFINED" # "CARTESIAN" "JOINT"
LAST_CURRENT_MODE = "UNDEFINED"
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

ser = serial.Serial("/dev/roverHeroRK", 115200)
is_resetting = False
last_data = Joy()
last_data.axes = [0, 0, 0, 0, 0, 0, 0, 0]
last_data.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


def joy_callback(data):
    # print(data.axes)
    global last_data, is_resetting, delta_x, delta_y, delta_z, delta_ang_x, delta_ang_y, delta_ang_z, button_1, button_2, button_1_last, button_2_last

 
    pulling_down_l2 = data.axes[2]
    pulling_down_r2 = data.axes[5]
    
    is_resetting = (pulling_down_l2 + pulling_down_r2 < -1.5)

    last_data = data
 
rospy.init_node("teleop_node")

publisher = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size = 10)
arm_publisher = rospy.Publisher('/manipulator_controller/command', JointTrajectory, queue_size=10) #

joy = rospy.Subscriber('/joy', Joy, joy_callback)

service_not_connected = True
rate = rospy.Rate(20)
comm_check_byte = 0

print("Waiting for toggle service")
rospy.wait_for_service('/hardware_interface/toggle_write')
toggle_service = rospy.ServiceProxy('/hardware_interface/toggle_write', Trigger)
last_state = "Now stopped writing to serial."
while not (last_data.axes[7] == 1 and last_data.buttons[3]):
    print("Please put the controller JOINT mode.")
    
while not rospy.is_shutdown():
    if (last_data.axes[6] == 1) and last_data.buttons[3]: # STEERING
        CURRENT_MODE = "STEERING"
        LAST_CURRENT_MODE = "JOINT"
    if (last_data.axes[7] == 1) and last_data.buttons[3]: # JOINT
        CURRENT_MODE = "JOINT"
        LAST_CURRENT_MODE = "JOINT"
    elif (last_data.axes[6] == -1) and last_data.buttons[3]: # CARTESIAN
        if service_not_connected:
            print("Waiting for servo service")
            rospy.wait_for_service('/servo_server/reset_target')
            reset_service = rospy.ServiceProxy('/servo_server/reset_target', Trigger)
            service_not_connected = False
            rospy.sleep(2)
        CURRENT_MODE = "CARTESIAN"
        if (LAST_CURRENT_MODE == "JOINT"):
            reset_service()
            print("reset target")
            rospy.sleep(2)
        LAST_CURRENT_MODE = "CARTESIAN"
  
    print("Current Mode: " + CURRENT_MODE)
    if CURRENT_MODE == "CARTESIAN":
        if last_state == "Now stopped writing to serial.":
            last_state = toggle_service().message
                
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        """	delta_x =
        delta_y = -data.axes[4] * data.buttons[1]
        delta_z = data.axes[4] * data.buttons[2]
        """
        msg.twist.linear.x =  deadband(last_data.axes[3], 0.15) * scaling_factor_linear
        msg.twist.linear.y = deadband(-last_data.axes[4], 0.15) * scaling_factor_linear
        msg.twist.linear.z = deadband(last_data.axes[1], 0.15) * scaling_factor_linear
        msg.twist.angular.x = delta_ang_x * scaling_factor_linear
        msg.twist.angular.y = delta_ang_y * scaling_factor_linear
        msg.twist.angular.z = delta_ang_z * scaling_factor_linear
        if (is_resetting):
            try:
                response = reset_service()
                print(response.message)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        print(msg)
        publisher.publish(msg)
  
    elif CURRENT_MODE == "JOINT":   
        if last_state == "Now writing to serial.":
            last_state = toggle_service().message
        
        comm_check_byte ^= 1
        message_to_send = ""
        message_to_send += "S"
        message_to_send += str(int(max(0, min(5 +
                                (last_data.axes[4] * last_data.buttons[0])*5, 9))))
        message_to_send += str(int(max(0, min(5 +
                                (last_data.axes[4] * last_data.buttons[1])*5, 9))))
        message_to_send += str(int(max(0, min(5 +
                                (last_data.axes[4] * last_data.buttons[2])*5, 9))))
        message_to_send += str(int(max(0, min(5 +
                                (last_data.axes[4] * last_data.buttons[3])*5, 9))))
        message_to_send += str(int(max(0, min(5 +
                                (last_data.axes[4] * last_data.buttons[4])*5, 9))))
        message_to_send += str(int(max(0, min(5 +
                                (-last_data.axes[4] * last_data.buttons[5])*5, 9))))
        message_to_send += str(int(max(0, min(5 +
                                (-last_data.axes[4] * last_data.buttons[6])*5, 9))))
        message_to_send += str(comm_check_byte)
        message_to_send += "F"
        if is_resetting:
            message_to_send = "RRRRR"
        print(message_to_send)
        ser.write(message_to_send)
    rate.sleep()

