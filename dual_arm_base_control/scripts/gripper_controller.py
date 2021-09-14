#!/usr/bin/env python  
# -*- coding:utf-8 -*-  

import rospy  
from sensor_msgs.msg import JointState
from joint_state_exchange.msg import RobotState
from std_msgs.msg import String
import time
import math


from wrist_gripper import WristGripperControl

fPWM = 50 # hz

servo_channel = [0,1,2,3]

wrist_gripper = WristGripperControl(servo_channel=servo_channel, is_debug=True)
wrist_gripper.all_ending_init()


def main():

    rospy.init_node('gripper_control_node', anonymous=True)
    
    robotStateSub()


def rad2angle(rad_val):
    # rad to angle cmd : 0-pi
    angle_val = (abs(rad_val)/math.pi) * 180
    return angle_val

# gripper control callback
def gripperControlCallback(armCmd,wrist_gripper,is_debug = True):

    # left wrist and right gripper angle control
    left_wrist_angle = rad2angle(armCmd.leftArmPosition[3])
    wrist_gripper.set_servo_angle(servo_idx = 0,angle = left_wrist_angle)
    time.sleep(0.02)
    left_gripper_angle = rad2angle(armCmd.leftArmPosition[4])
    wrist_gripper.set_servo_angle(servo_idx = 1, angle = left_gripper_angle)
    time.sleep(0.02)
    # right wrist and right gripper angle control
    right_wrist_angle = rad2angle(armCmd.rightArmPosition[3])
    wrist_gripper.set_servo_angle(servo_idx = 2,angle = right_wrist_angle)
    time.sleep(0.02)
    right_gripper_angle = rad2angle(armCmd.rightArmPosition[4])
    wrist_gripper.set_servo_angle(servo_idx = 3,angle = right_gripper_angle)
    time.sleep(0.02)

    # armCmd.leftArmName[4] = "left_arm_gripper"
    if is_debug:
        rospy.loginfo(rospy.get_caller_id() + "gripper_control_node heard %s, %s", armCmd.leftArmName[3], armCmd.leftArmName[4])
        rospy.loginfo(rospy.get_caller_id() + "gripper_control_node heard %s, %s", armCmd.rightArmName[3], armCmd.rightArmName[4])
        
        rospy.loginfo(rospy.get_caller_id() + "left gripper_control cmd %f, %f",armCmd.leftArmPosition[3],armCmd.leftArmPosition[4])
        rospy.loginfo(rospy.get_caller_id() + "Right gripper_control cmd %f, %f",armCmd.rightArmPosition[3],armCmd.rightArmPosition[4])
        
def robotStateSub():
    rospy.Subscriber("/robot_arm_state", RobotState,callback=gripperControlCallback())

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass

