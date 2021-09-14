#!/usr/bin/env python
# coding:utf-8

# 上面指定编码utf-8，使python能够识别中文

# 加载必需模块，注意service模块的加载方式，from 包名.srv import *
# 其中srv指的是在包根目录下的srv文件夹，也即srv模块
import rospy
from dual_arm_init.srv import *

def server_srv():
    # 初始化节点，命名为 "init_server"
    rospy.init_node("init_server_node")
    # 定义service的server端，service名称为“arm_pose_init”， service类型为poseInit
    # 收到的request请求信息将作为参数传递给handle_function进行处理
    init_srv = rospy.Service("arm_pose_init",poseInit, handle_function)
    rospy.loginfo("Ready to handle the request:")
    # 阻塞程序结束
    rospy.spin()

# Define the handle function to handle the request inputs
def handle_function(req):
    
    # 注意我们是如何调用request请求内容的，与前面client端相似，都是将其认为是一个对象的属性，通过对象调用属性，在我们定义
    # 的Service_demo类型的service中，request部分的内容包含两个变量，一个是字符串类型的name，另外一个是整数类型的age
    rospy.loginfo( 'Request from robot arm init client {}'.format(req.flag))
    # 返回一个Service_demoResponse实例化对象，其实就是返回一个response的对象，其包含的内容为我们再Service_demo.srv中定义的
    # response部分的内容，我们定义了一个string类型的变量，因此，此处实例化时传入字符串即可
    if req.flag:

        
        is_inited = True 
        rospy.loginfo("Hi {} I' server!".format(is_inited))
        
    else:
        is_inited = False
        rospy.loginfo("Hi {} I' server!".format(is_inited))
    return poseInitResponse(is_inited)

# 如果单独运行此文件，则将上面定义的server_srv作为主函数运行
if __name__=="__main__":
    server_srv()
