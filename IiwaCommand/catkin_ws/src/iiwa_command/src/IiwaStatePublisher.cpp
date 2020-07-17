#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/TwistStamped.h"
#include <iiwa_command/MoveLAction.h>
#include <actionlib/server/simple_action_server.h>
#include <Eigen/Dense>
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
    //Iiwa state subscriber
    ros::Subscriber iiwa_state_sub;
    sensor_msgs::JointState joint_state;
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
};
