#!/usr/bin/env python
# coding:utf-8

# 上面的第二句指定编码类型为utf-8，是为了使python能够识别中文

# 加载所需模块
import rospy
from dual_arm_init.srv import *

def client_srv():
    rospy.init_node('init_client_node')
    # 等待有可用的服务 "arm_pose_init"
    rospy.wait_for_service("arm_pose_init")
    try:
        # 定义service客户端，service名称为“arm_pose_init”，service类型为poseInit
        arm_init_client = rospy.ServiceProxy("arm_pose_init",poseInit)

        # 注意，此处发送的request内容与service文件中定义的request部分的属性是一致的
        flag = True
        resp = arm_init_client(flag)
        if resp.is_inited:
            rospy.loginfo("Message From server:{}".format(resp.is_inited))
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s"%e)

# 如果单独运行此文件，则将上面函数client_srv()作为主函数运行
if __name__=="__main__":
    client_srv()
