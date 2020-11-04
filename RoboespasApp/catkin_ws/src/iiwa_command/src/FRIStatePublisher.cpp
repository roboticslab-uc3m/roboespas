#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <eigen3/Eigen/Dense>
#include "FRIClient.cpp"


using namespace std;

class FRIStatePublisher
{
  private:
    ros::NodeHandle nh;
    sensor_msgs::JointState joint_state;
    ros::Publisher js_pub;
    FRIClient* friClient;

    //Fix state for each timer callback
    double q_read[7];
    double qtorque_read[7];
    ros::Time curr_time_stamp;
    double control_step_size;
  public:
    FRIStatePublisher(std::string name, FRIClient* friClient_)
    {
        friClient = friClient_;
        js_pub = nh.advertise<sensor_msgs::JointState>("/iiwa_command/joint_state", 1000, false);
        joint_state.name = {"J1","J2","J3","J4","J5","J6","J7"};
        ROS_INFO("Publisher /iiwa_command/joint_state started");
    }
    void PublishState()
    {
        control_step_size = friClient->control_step_size;
        nh.setParam("/iiwa_command/control_step_size", control_step_size);
        memcpy(q_read, friClient->q_read, LBRState::NUMBER_OF_JOINTS * sizeof(double));
        memcpy(qtorque_read, friClient->qtorque_read, LBRState::NUMBER_OF_JOINTS * sizeof(double));
        curr_time_stamp = friClient->curr_time_stamp;
        //Update joint_state
        joint_state.position.clear();
        for (int i=0; i<7; i++)
        {
            joint_state.position.push_back(q_read[i]);
        }
        joint_state.effort.clear();
        for (int i=0; i<7; i++)
        {
            joint_state.effort.push_back(qtorque_read[i]);
        }
        //Update stamp
        joint_state.header.stamp=curr_time_stamp;
        js_pub.publish(joint_state); //Just publish the whole joint_state when a new joint_position arrives
    }
    void PublishLBRState()
    {

    }
};
