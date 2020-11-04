#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include "ros/ros.h"
#include <iiwa_command/MoveJAction.h>
#include <eigen3/Eigen/Dense>
#include <ros/xmlrpc_manager.h>
#include <iiwa_msgs/SetPathParameters.h>

using namespace std;

class StackConfigurationSubscriber
{
    protected:
    ros::NodeHandle nh;
    ros::ServiceClient pathParam_cli;
    public:
    double velocity_param;
    double acceleration_param;
    double jerk_param;
    StackConfigurationSubscriber(std::string name)
    {
        //Subscribe to parameter changes
        ros::XMLRPCManager::instance()->unbind("paramUpdate");
        XmlRpc::XmlRpcValue params, result, payload;
        ros::XMLRPCManager::instance()->bind("paramUpdate", boost::bind(&StackConfigurationSubscriber::callback_paramChanged, this, params, result));
        params[0] = ros::this_node::getName();
        params[1] = ros::XMLRPCManager::instance()->getServerURI();
        params[2] = ros::names::resolve(std::string("/iiwa_command/velocity"));//, std::string("/iiwa_command/acceleration"));//, std::string("/iiwa_command/jerk"));
        if (ros::master::execute("subscribeParam", params, result, payload, false))
        {
            //ROS_INFO("Subscribed to parameter velocity.");
        }
        else
        {
            ROS_ERROR("Failed to subscribe to the parameter /iiwa_command/velocity.");
        }
        params[2] = ros::names::resolve(std::string("/iiwa_command/acceleration"));
        if (ros::master::execute("subscribeParam", params, result, payload, false))
        {
            //ROS_INFO("Subscribed to parameter accelerations.");
        }
        else
        {
            ROS_ERROR("Failed to subscribe to the parameter /iiwa_command/acceleration.");
        }
        params[2] = ros::names::resolve(std::string("/iiwa_command/jerk"));
        if (ros::master::execute("subscribeParam", params, result, payload, false))
        {
            //ROS_INFO("Subscribed to parameter jerk.");
        }
        else
        {
            ROS_ERROR("Failed to subscribe to the parameter /iiwa_command/jerk.");
        }
        ROS_INFO("Subscribed to /iiwa_command/velocity, /iiwa_command/acceleration and /iiwa_command/jerk.");
        //Create service client
        pathParam_cli = nh.serviceClient<iiwa_msgs::SetPathParameters>("/iiwa/configuration/pathParameters");
    }
    void callback_paramChanged(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
    {
        //Read parameters
        if (!nh.getParam("/iiwa_command/velocity", velocity_param))
        {
            ROS_ERROR("Failed to read '/iiwa_command/velocity' on param server");
        }
        if (!nh.getParam("/iiwa_command/acceleration", acceleration_param))
        {
            ROS_ERROR("Failed to read '/iiwa_command/acceleration' on param server");
        }
        if (!nh.getParam("/iiwa_command/jerk", jerk_param))
        {
            ROS_ERROR("Failed to read '/iiwa_command/jerk' on param server");
        }
        //Call service
        iiwa_msgs::SetPathParameters pathParam_msg;
        pathParam_msg.request.joint_relative_velocity = velocity_param;
        pathParam_msg.request.joint_relative_acceleration = acceleration_param;
        pathParam_msg.request.override_joint_acceleration = jerk_param;
        if (pathParam_cli.call(pathParam_msg), 2)
        {
            ROS_INFO("Changed velocity/acceleration/jerk to %f/%f/%f", velocity_param, acceleration_param, jerk_param);
        }
        else
        {
            ROS_ERROR("Error calling SetPathParameters, please restart communication, move the robot to a position far away from workspace limits, and retry.");
        }
    }
};
