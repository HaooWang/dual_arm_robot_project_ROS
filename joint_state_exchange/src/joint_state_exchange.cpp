#include "ros/ros.h"
#include "joint_state_exchange/RobotState.h"
#include "std_msgs/String.h"
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
    StateSubAndPub(){
    // define Topic name: /robot_state, msg tpye: /RobotState
    // publisher:pub_, topic: /robot_arm_state, RobotState
    // nodehandle: n_  
    pub_ = n_.advertise<joint_state_exchange::RobotState>(
                            "/robot_arm_state",
                            frequence);

    // define Subscriber:/joint_state_publisher_gui ->  /joint_state_publisher Topic 
    // Type: sensor_msgs/JointState 
    sub_ = n_.subscribe("/joint_states",
                        1000,
                        &StateSubAndPub::chatterCallback,
                        this);   
    } //默认构造

    // StateSubAndPub(string topic_name);  // 自定义构造函数

    void chatterCallback(const sensor_msgs::JointState::ConstPtr &input_msg)
    {
        // define the temp output msg: type /RobotState
        joint_state_exchange::RobotState temp_pub_msg;

        temp_pub_msg.leftArmName.resize(5); // define the left arm joints name string len
        temp_pub_msg.rightArmName.resize(5); // define the right arm joints name string len

        temp_pub_msg.leftArmPosition.resize(5);
        temp_pub_msg.rightArmPosition.resize(5);

        temp_pub_msg.leftArmVelocity.resize(5);
        temp_pub_msg.rightArmVelocity.resize(5);

        // exchange the msg 
        temp_pub_msg.header = input_msg->header;
        // joint name:left arm [0]-[3],dof4
        temp_pub_msg.leftArmName[0] = input_msg->name[0];
        temp_pub_msg.leftArmName[1] = input_msg->name[1]; 
        temp_pub_msg.leftArmName[2] = input_msg->name[4];
        temp_pub_msg.leftArmName[3] = input_msg->name[5];
        temp_pub_msg.leftArmName[4] = "left_arm_gripper";

        // joint name: right arm [0]-[3],dof4
        temp_pub_msg.rightArmName[0] = input_msg->name[6];
        temp_pub_msg.rightArmName[1] = input_msg->name[7];
        temp_pub_msg.rightArmName[2] = input_msg->name[10];
        temp_pub_msg.rightArmName[3] = input_msg->name[11];
        temp_pub_msg.rightArmName[4] = "right_arm_gripper";
        
        //joint position : angle of each servo
        temp_pub_msg.leftArmPosition[0] = input_msg->position[0];
        temp_pub_msg.leftArmPosition[1] = input_msg->position[1];
        temp_pub_msg.leftArmPosition[2] = input_msg->position[4];
        temp_pub_msg.leftArmPosition[3] = input_msg->position[5];
        temp_pub_msg.leftArmPosition[4] = input_msg->position[2];
        
        temp_pub_msg.rightArmPosition[0] = input_msg->position[6];
        temp_pub_msg.rightArmPosition[1] = input_msg->position[7];
        temp_pub_msg.rightArmPosition[2] = input_msg->position[10];
        temp_pub_msg.rightArmPosition[3] = input_msg->position[11];
        temp_pub_msg.rightArmPosition[4] = input_msg->position[8];


        temp_pub_msg.leftArmVelocity[0] = 0;
        temp_pub_msg.leftArmVelocity[1] = 0;
        temp_pub_msg.leftArmVelocity[2] = 0;
        temp_pub_msg.leftArmVelocity[3] = 0;
        temp_pub_msg.leftArmVelocity[4] = 0;

        temp_pub_msg.rightArmVelocity[0] = 0;
        temp_pub_msg.rightArmVelocity[1] = 0;
        temp_pub_msg.rightArmVelocity[2] = 0;
        temp_pub_msg.rightArmVelocity[3] = 0;
        temp_pub_msg.rightArmVelocity[4] = 0;
        

        pub_.publish(temp_pub_msg);
        ROS_INFO("I heard: [%s] and published.", input_msg->name[0].c_str());
    }

private:

    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    int frequence = 2;  // 消息发布频率
};


// StateSubAndPub::StateSubAndPub(){
//     // define Topic name: /robot_state, msg tpye: /RobotState
//     // publisher:pub_, topic: /robot_arm_state, RobotState
//     // nodehandle: n_  
//     pub_ = n_.advertise<joint_state_exchange::RobotState>(
//                             "/robot_arm_state",
//                             frequence);

//     // define Subscriber:/joint_state_publisher_gui ->  /joint_state_publisher Topic 
//     // Type: sensor_msgs/JointState 
//     sub_ = n_.subscribe("/joint_states",
//                         1000,
//                         &StateSubAndPub::chatterCallback,
//                         this);                                                                                                                                                                                  
// }

// StateSubAndPub::chatterCallback(
//     const sensor_msgs::JointState::ConstPtr &msg)
// {
//     // define the temp output msg: type /RobotState
//     joint_state_exchange::RobotState temp_pub_msg;

//     temp_pub_msg.leftArmName.resize(4); // define the left arm joints name string len
//     temp_pub_msg.rightArmName.resize(4); // define the right arm joints name string len

//     temp_pub_msg.leftArmPosi                                                                                                        tion.resize(4);
//     temp_pub_msg.rightArmPosition.resize(4);

//      // exchange the msg 
// 	temp_pub_msg.header = msg->header;
//     // joint name:left arm [0]-[3],dof4
//     temp_pub_msg.leftArmName[0] = msg->name[0];
//     temp_pub_msg.leftArmName[1] = msg->name[1]; 
//     temp_pub_msg.leftArmName[2] = msg->name[4];
//     temp_pub_msg.leftArmName[3] = msg->name[5];
//     // joint name: right arm [0]-[3],dof4
//     temp_pub_msg.rightArmName[0] = msg->name[6];
//     temp_pub_msg.rightArmName[1] = msg->name[7];
//     temp_pub_msg.rightArmName[2] = msg->name[10];
//     temp_pub_msg.rightArmName[3] = msg->name[11];
    
//     //joint position : angle of each servo
//     temp_pub_msg.leftArmPosition[0] = msg->position[0];
//     temp_pub_msg.leftArmPosition[1] = msg->position[1];
//     temp_pub_msg.leftArmPosition[2] = msg->position[4];
//     temp_pub_msg.leftArmPosition[3] = msg->position[5];
    
//     temp_pub_msg.rightArmPosition[0] = msg->position[6];
//     temp_pub_msg.rightArmPosition[1] = msg->position[7];
//     temp_pub_msg.rightArmPosition[2] = msg->position[10];
//     temp_pub_msg.rightArmPosition[3] = msg->position[11];

//     pub_publish(temp_pub_msg);
//     ROS_INFO("I heard: [%s] and published.", msg->header.c_str());

// }

int main(int argc, char *argv[])
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "joint_state_exchange_pub");


	/**
	 * The subscribe() call is how you tell ROS that you want to receive messages
	 * on a given topic.  This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing.  Messages are passed to a callback function, here
	 * called chatterCallback.  subscribe() returns a Subscriber object that you
	 * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	 * object go out of scope, this callback will automatically be unsubscribed from
	 * this topic.
	 *
	 * The second parameter to the subscribe() function is the size of the message
	 * queue.  If messages are arriving faster than they are being processed, this
	 * is the number of messages that will be buffered up before beginning to throw
	 * away the oldest ones.
	 */
	
    StateSubAndPub SSPObj;

	ros::spin();

	return 0;
}