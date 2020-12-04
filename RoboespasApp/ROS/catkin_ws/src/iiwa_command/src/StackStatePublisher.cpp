#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <eigen3/Eigen/Dense>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointTorque.h>

using namespace std;

class StackStatePublisher
{
    protected:
    ros::NodeHandle nh;
    ros::Subscriber q_sub;
    ros::Subscriber qdot_sub;
    ros::Subscriber qtorque_sub;
    sensor_msgs::JointState joint_state;
    ros::Publisher js_pub;
    public:

    StackStatePublisher(std::string name)
    {
        q_sub = nh.subscribe("/iiwa/state/JointPosition", 1000, &StackStatePublisher::callback_qSub, this);
        qdot_sub = nh.subscribe("/iiwa/state/JointVelocity", 1000, &StackStatePublisher::callback_qdotSub, this);
        qtorque_sub = nh.subscribe("/iiwa/state/JointTorque", 1000, &StackStatePublisher::callback_qtorqueSub, this);
        js_pub = nh.advertise<sensor_msgs::JointState>("/iiwa_command/joint_state", 1000, false);
        joint_state.name = {"J1","J2","J3","J4","J5","J6","J7"};
        ROS_INFO("Publisher /iiwa_command/joint_state started");
    }
    void callback_qSub(const iiwa_msgs::JointPosition& q_msg)
    {
        //Update joint_state
        sensor_msgs::JointState js = joint_state; //To avoid the joint_state be
        js.position.clear();
        js.position.push_back(q_msg.position.a1);
        js.position.push_back(q_msg.position.a2);
        js.position.push_back(q_msg.position.a3);
        js.position.push_back(q_msg.position.a4);
        js.position.push_back(q_msg.position.a5);
        js.position.push_back(q_msg.position.a6);
        js.position.push_back(q_msg.position.a7);
        //Update stamp
        js.header=q_msg.header;
        joint_state = js;
        js_pub.publish(joint_state); //Just publish the whole joint_state when a new joint_position arrives
    }
    void callback_qdotSub(const iiwa_msgs::JointVelocity& qdot_msg)
    {
        //Update velocity
        sensor_msgs::JointState js = joint_state;
        js.velocity.clear();
        js.velocity.push_back(qdot_msg.velocity.a1);
        js.velocity.push_back(qdot_msg.velocity.a2);
        js.velocity.push_back(qdot_msg.velocity.a3);
        js.velocity.push_back(qdot_msg.velocity.a4);
        js.velocity.push_back(qdot_msg.velocity.a5);
        js.velocity.push_back(qdot_msg.velocity.a6);
        js.velocity.push_back(qdot_msg.velocity.a7);
        joint_state = js;
        //Do not update stamp because the stamp received here is wrong
    }
    void callback_qtorqueSub(const iiwa_msgs::JointTorque& qtorque_msg)
    {
        //Update torque
        sensor_msgs::JointState js = joint_state;
        js.effort.clear();
        js.effort.push_back(qtorque_msg.torque.a1);
        js.effort.push_back(qtorque_msg.torque.a2);
        js.effort.push_back(qtorque_msg.torque.a3);
        js.effort.push_back(qtorque_msg.torque.a4);
        js.effort.push_back(qtorque_msg.torque.a5);
        js.effort.push_back(qtorque_msg.torque.a6);
        js.effort.push_back(qtorque_msg.torque.a7);
        //Update stamp
        js.header = qtorque_msg.header;
        joint_state=js;
    }
};
