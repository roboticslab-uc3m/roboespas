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
#include <iiwa_command/CaptureStart.h>
#include <iiwa_command/CaptureStop.h>

using namespace std;

class StackCaptureService
{
    protected:
    ros::NodeHandle nh;
    ros::Subscriber q_sub;
    ros::Subscriber qdot_sub;
    ros::Subscriber qtorque_sub;
    sensor_msgs::JointState joint_state;
    ros::ServiceServer capture_start_srv;
    ros::ServiceServer capture_stop_srv;
    vector<sensor_msgs::JointState> trajectory_read;
    bool bCapture=false;
    public:

    StackCaptureService(std::string name)
    {
        ROS_INFO("Services %s offered", name.c_str());
        q_sub = nh.subscribe("/iiwa/state/JointPosition", 1000, &StackCaptureService::callback_qSub, this);
        qdot_sub = nh.subscribe("/iiwa/state/JointVelocity", 1000, &StackCaptureService::callback_qdotSub, this);
        qtorque_sub = nh.subscribe("/iiwa/state/JointTorque", 1000, &StackCaptureService::callback_qtorqueSub, this);
        capture_start_srv = nh.advertiseService("/iiwa_command/capture/start", &StackCaptureService::callback_captureStart, this);
        capture_stop_srv = nh.advertiseService("/iiwa_command/capture/stop", &StackCaptureService::callback_captureStop, this);
        ROS_INFO("Services /iiwa_command/capture/start and /iiwa_command/capture/stop offered");
        joint_state.name = {"J1","J2","J3","J4","J5","J6","J7"};
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
        if (bCapture)
        {
            trajectory_read.push_back(joint_state);
        }
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
    bool callback_captureStart(iiwa_command::CaptureStart::Request &req, iiwa_command::CaptureStart::Response &res)
    {
        ROS_INFO("Started capture...");
        trajectory_read.clear();
        bCapture=true;
        res.success=true;
        return true;
    }
    bool callback_captureStop(iiwa_command::CaptureStop::Request &req, iiwa_command::CaptureStop::Response &res)
    {
        bCapture = false;
        res.trajectory_read = trajectory_read;
        res.success=true;
        ROS_INFO("Capture finished and returned.");
        return true;
    }
};
