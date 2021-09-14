#!/usr/bin/env python  
# -*- coding:utf-8 -*-  

import rospy  
from sensor_msgs.msg import JointState
from joint_state_exchange.msg import RobotState
from std_msgs.msg import String

def main():
    rospy.init_node('arm_control_node', anonymous=True)

    robotStateSub()


# gripper control callback
def gripperControlCallback(armCmd):
    
    rospy.loginfo(rospy.get_caller_id() + "gripper_control_node heard %s", armCmd.leftArmName[0])
    # armCmd.leftArmName[4] = "left_arm_gripper"
    # temp_pub_msg.leftArmPosition[4] = input_msg->position[2];
    # left arm gripper 
    rospy.loginfo(rospy.get_caller_id() + "left gripper_control cmd %f",armCmd.leftArmPosition[1])
    
    rospy.loginfo(rospy.get_caller_id() + "right gripper_control cmd %f",armCmd.leftArmPosition[1])
    

def robotStateSub():
    rospy.Subscriber("/robot_arm_state", RobotState,callback=gripperControlCallback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInitException:
        pass

