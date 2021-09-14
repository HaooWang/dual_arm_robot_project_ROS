# ifndef STATESUBANDPUB_H
# define STATESUBANDPUB_H


#include "ros/ros.h"
#include "RobotState.h"
#include "string"
#include "sensor_msgs/JointState.h"


class StateSubAndPub{

/**
 *订阅双臂机器人/joint_state_publisher_gui ->  /joint_state_publisher Topic 
 *Type: sensor_msgs/JointState
 *
 *Publish:
 * 双臂机器人关节状态信息，时刻k 
 *  (http://haowang-codingspace.local:33461/)
 */

public:
    StateSubAndPub();  //默认构造

    // StateSubAndPub(string topic_name);  // 自定义构造函数

    void chatterCallback(const sensor_msgs::JointState::ConstPtr &input_msg);
    ~StateSubAndPub();  //默认析构函数

private:

    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    const ros::int8 frequence = 2;  // 消息发布频率
};

#endif  // END

