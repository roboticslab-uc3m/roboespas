#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <iiwa_command/IiwaCommandAction.h>
#include <actionlib/server/simple_action_server.h>
#include <Eigen/Dense>
#include <math.h>

using namespace std;

class IiwaCommandNode
{
    protected:
    ros::NodeHandle nh;
    //Iiwa command action server variables
    actionlib::SimpleActionServer<iiwa_command::IiwaCommandAction> as;
    iiwa_command::IiwaCommandFeedback as_feedback;
    iiwa_command::IiwaCommandResult as_result;
    //Iiwa gazebo state subscriber
    ros::Subscriber iiwa_gazebo_state_sub;
    //Iiwa gazebo command publisher
    ros::Publisher iiwa_gazebo_command_pub;
    sensor_msgs::JointState curr_joint_state;

    public:

    IiwaCommandNode(std::string name) :
    as(nh, name, boost::bind(&IiwaCommandNode::callback_iiwa_command, this, _1), false) //Create the action server
    {
	    ROS_INFO("Node registered as %s\n", name.c_str());
        as.start();
        ROS_INFO("Action server %s started", name.c_str());
        iiwa_gazebo_command_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/iiwa_gazebo/joint_command", 1000, false);
        iiwa_gazebo_state_sub = nh.subscribe("/iiwa_gazebo/joint_state", 1000, &IiwaCommandNode::callback_iiwa_gazebo_state, this);
    }
    void callback_iiwa_gazebo_state(const sensor_msgs::JointState& iiwa_gazebo_state_msg)
    {
        curr_joint_state=iiwa_gazebo_state_msg;
    }
    void callback_iiwa_command(const iiwa_command::IiwaCommandGoalConstPtr &goal)
    {
        //PD Gains
        Eigen::VectorXd weights(7);
        Eigen::VectorXd Kp(7);
        Eigen::VectorXd Kd(7);
        weights << 0.3, 0.8, 0.6, 0.6, 0.3, 0.2, 0.1;
        Kp = 100*weights;
        Kd = 2*weights;

        //Variables used inside 
        ros::Time tStartTraj;
        std::vector<trajectory_msgs::JointTrajectoryPoint> points = goal->trajectory_desired.points;

        //Variables returned
        trajectory_msgs::JointTrajectory trajectory_commanded;
        std::vector<sensor_msgs::JointState> trajectory_joint_state;

        for (int i=0; i < points.size()-1; i++)
        {
            //Get current joint state and save it in trajectory_followed
            sensor_msgs::JointState js=curr_joint_state;
            trajectory_joint_state.push_back(js);
            //Fill tstartTraj if i==0
            if (i==0)
            {
                tStartTraj=js.header.stamp;
            }
            //Calculate current iteration expected duration depending on points
            ros::Duration it_dur=points[i+1].time_from_start - points[i].time_from_start;
            //Get current index depending on how much time has passed since the beginning
            int h=ceil((js.header.stamp.toSec()-tStartTraj.toSec() + 1e-8)/it_dur.toSec())-1;
            //Get variables used as Eigen vectors or ros::Durations
            // - Current q_curr / qdot_curr
            Eigen::VectorXd q_curr = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(js.position.data(), js.position.size());
            Eigen::VectorXd qdot_curr = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned> (js.velocity.data(), js.velocity.size());
            // - Desired tau_des, q_des, qdot_des
            Eigen::VectorXd tau_des = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(points[h].effort.data(), points[h].effort.size());
            Eigen::VectorXd q_des = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned> (points[h].positions.data(), points[h].positions.size());
            Eigen::VectorXd qdot_des = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned> (points[h].velocities.data(), points[h].velocities.size());
            //Calculate tau to correct the desired tau
            Eigen::VectorXd tau_added = Kp.cwiseProduct(q_des - q_curr) + Kd.cwiseProduct(qdot_des - qdot_curr);
            //Calculate tau to send and it to point_command
            Eigen::VectorXd tau_total = tau_des + tau_added;
            std::vector<double> tau_total_vec(tau_total.data(), tau_total.data() + tau_total.size());
            trajectory_msgs::JointTrajectoryPoint point_command;
            point_command.effort = tau_total_vec;
            trajectory_commanded.points.push_back(point_command);

            //Cout calculations
/*
            std::cout << "i " << i << std::endl;
            std::cout << "tStartTraj: " << tStartTraj.toSec() << std::endl;
            std::cout << "tCurr: " << js.header.stamp.toSec() << std::endl;
            std::cout << "it_dur: " << it_dur.toSec() << std::endl;
            std::cout << "h: " << h << std::endl;
            std::cout << "q_curr: " << q_curr.transpose() << std::endl << "qdot_curr: " << qdot_curr.transpose() << std::endl;
            std::cout << "tau_des: " << tau_des.transpose() << std::endl << "q_des: " << q_des.transpose() << std::endl << "qdot_des: " << qdot_des.transpose() << std::endl; 
            std::cout << "tau_added: " << tau_added.transpose() << std::endl;
            std::cout << "tau_total: " << tau_total.transpose() << std::endl;*/


            //Publish feedback                     
            //as_feedback.joint_state = curr_joint_state;
            //as.publishFeedback(as_feedback);

            //Send point_commanded
            iiwa_gazebo_command_pub.publish(point_command);

            //Calculate time to sleep and sleep
            /*ros::Duration tProcess = curr_joint_state.header.stamp-js.header.stamp;
            ros::Duration tSleep = it_dur-tProcess;
            std::cout << "time process: " << tProcess.toSec() << std::endl;
            std::cout << "time sleep: " << tSleep.toSec() << std::endl;*/
            ros::Duration(0.000001).sleep();
        }
        
        as_result.trajectory_joint_state=trajectory_joint_state;
        as_result.trajectory_commanded=trajectory_commanded;
        as.setSucceeded(as_result);
    }
};


int main(int argc, char **argv)
{
	ros::init(argc,argv, "iiwa_command");	
    IiwaCommandNode iiwacommand(ros::this_node::getName());

	bool success = true;
	while (ros::ok())
	{
		ros::spinOnce();
	}
	return 0;
}
