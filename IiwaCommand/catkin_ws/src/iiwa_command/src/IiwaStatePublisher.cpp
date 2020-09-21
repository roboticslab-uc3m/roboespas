#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "iiwa_msgs/JointVelocity.h"
#include "iiwa_msgs/JointTorque.h"
#include "geometry_msgs/TwistStamped.h"
#include <iiwa_command/MoveLAction.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <cmath>
#include "IiwaTrajectoryGeneration.cpp"
#include "IiwaScrewTheory.cpp"

using namespace Eigen;
using namespace std;

class IiwaStatePublisher
{
    protected:
    ros::NodeHandle nh;
    //Iiwa gazebo/fri state subscriber
    ros::Subscriber iiwa_state_sub;
    //Iiwa stack state Subscriber
    ros::Subscriber iiwa_stack_q_sub;
    ros::Subscriber iiwa_stack_qdot_sub;
    ros::Subscriber iiwa_stack_qtorque_sub;
    sensor_msgs::JointState iiwa_stack_joint_state;
    //Iiwa gazebo/fri state publisher
    ros::Publisher iiwa_state_pub;
    string robot_mode;

    public:

    IiwaStatePublisher(string name)
    {
        //Simple node that redirects the joint state from iiwa_fri/joint_state or iiwa_gazebo/joint_state to iiwa_command/joint_state
        ROS_INFO("Publisher %s started", name.c_str());
        //Read robot mode
        if(!nh.getParam("/iiwa_command/robot_mode", robot_mode))
        {
            ROS_ERROR("Failed to read '/iiwa_command/robot_mode' on param server");
        }
        //Initializate topics depending on robot_mode
        if (strcmp(robot_mode.c_str(), "gazebo")==0)
        {
            iiwa_state_sub = nh.subscribe("/iiwa_gazebo/joint_state", 1000, &IiwaStatePublisher::callback_iiwa_state, this);
        }
        else if (strcmp(robot_mode.c_str(), "fri")==0)
        {
            iiwa_state_sub = nh.subscribe("/iiwa_fri/joint_state", 1000, &IiwaStatePublisher::callback_iiwa_state, this);
        }
        else if (strcmp(robot_mode.c_str(), "iiwa_stack")==0)
        {
            iiwa_stack_q_sub = nh.subscribe("/iiwa/state/JointPosition", 1000, &IiwaStatePublisher::callback_q_iiwa_stack, this);
            iiwa_stack_qdot_sub = nh.subscribe("/iiwa/state/JointVelocity", 1000, &IiwaStatePublisher::callback_qdot_iiwa_stack, this);
            iiwa_stack_qtorque_sub = nh.subscribe("/iiwa/state/JointTorque", 1000, &IiwaStatePublisher::callback_qtorque_iiwa_stack, this);

        }
        else
        {
            ROS_ERROR("Not implemented yet");
        }
        iiwa_state_pub = nh.advertise<sensor_msgs::JointState>("/iiwa_command/joint_state", 1000, false);
    }
    void callback_iiwa_state(const sensor_msgs::JointState& iiwa_state_msg)
    {
        iiwa_state_pub.publish(iiwa_state_msg);
    }
    void callback_q_iiwa_stack(const iiwa_msgs::JointPosition& q_msg)
    {
        //Update position
        iiwa_stack_joint_state.position.clear();
        iiwa_stack_joint_state.position.push_back(q_msg.position.a1);
        iiwa_stack_joint_state.position.push_back(q_msg.position.a2);
        iiwa_stack_joint_state.position.push_back(q_msg.position.a3);
        iiwa_stack_joint_state.position.push_back(q_msg.position.a4);
        iiwa_stack_joint_state.position.push_back(q_msg.position.a5);
        iiwa_stack_joint_state.position.push_back(q_msg.position.a6);
        iiwa_stack_joint_state.position.push_back(q_msg.position.a7);
        //Update stamp
        iiwa_stack_joint_state.header=q_msg.header;
        //Publish
        iiwa_state_pub.publish(iiwa_stack_joint_state);
    }
    void callback_qdot_iiwa_stack(const iiwa_msgs::JointVelocity& qdot_msg)
    {
        //Update velocity
        iiwa_stack_joint_state.velocity.clear();
        iiwa_stack_joint_state.velocity.push_back(qdot_msg.velocity.a1);
        iiwa_stack_joint_state.velocity.push_back(qdot_msg.velocity.a2);
        iiwa_stack_joint_state.velocity.push_back(qdot_msg.velocity.a3);
        iiwa_stack_joint_state.velocity.push_back(qdot_msg.velocity.a4);
        iiwa_stack_joint_state.velocity.push_back(qdot_msg.velocity.a5);
        iiwa_stack_joint_state.velocity.push_back(qdot_msg.velocity.a6);
        iiwa_stack_joint_state.velocity.push_back(qdot_msg.velocity.a7);
        //Do not update stamp bc stamp provided by iiwa_stack is wrong
        //Publish
        iiwa_state_pub.publish(iiwa_stack_joint_state);
    }
    void callback_qtorque_iiwa_stack(const iiwa_msgs::JointTorque& qtorque_msg)
    {
        //Update torque
        iiwa_stack_joint_state.effort.clear();
        iiwa_stack_joint_state.effort.push_back(qtorque_msg.torque.a1);
        iiwa_stack_joint_state.effort.push_back(qtorque_msg.torque.a2);
        iiwa_stack_joint_state.effort.push_back(qtorque_msg.torque.a3);
        iiwa_stack_joint_state.effort.push_back(qtorque_msg.torque.a4);
        iiwa_stack_joint_state.effort.push_back(qtorque_msg.torque.a5);
        iiwa_stack_joint_state.effort.push_back(qtorque_msg.torque.a6);
        iiwa_stack_joint_state.effort.push_back(qtorque_msg.torque.a7);
        //Update stamp
        iiwa_stack_joint_state.header = qtorque_msg.header;
        //Publish
        iiwa_state_pub.publish(iiwa_stack_joint_state);
    }
};
