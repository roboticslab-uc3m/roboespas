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
    sensor_msgs::JointState read_joint_state;

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
        read_joint_state=iiwa_gazebo_state_msg;
    }
    void callback_iiwa_command(const iiwa_command::IiwaCommandGoalConstPtr &goal)
    {
        //Data
        double  sample_time=0.001;
        //PD Gains
        Eigen::VectorXd weights(7);
        Eigen::VectorXd Kp(7);
        Eigen::VectorXd Kd(7);
        weights << 0.3, 0.8, 0.6, 0.6, 0.3, 0.2, 0.1;
        Kp = 100*weights;
        Kd = 2*weights;

        //Variables used
        std::vector<trajectory_msgs::JointTrajectoryPoint> goal_points = goal->trajectory_desired.points;

        //Variables returned
        trajectory_msgs::JointTrajectory trajectory_commanded;
        std::vector<sensor_msgs::JointState> trajectory_joint_state;

        //Initialize the index that will contain the current index of the trajectory depending on the time that has passed
        int i=0;

        //Get the time at which the trajectory starts being sent
        ros::Time tStartTraj = ros::Time::now();//read_joint_state.header.stamp;
        while (i<goal_points.size())
        {
            //read_joint_state may change during this iteration, save it in a different variable to fix it
            sensor_msgs::JointState joint_state=read_joint_state;
            double time_from_start=(ros::Time::now()-tStartTraj).toSec(); 
            //Get the index in the goal_points vector corresponding to the current time
            i = time_from_start/sample_time;
            //If the index is outside the goal_points vector, exit the while loop
            if (i>=goal_points.size()) 
                break;
            //Get variables used as Eigen vectors / ROS durations
            // - Current q_curr / qdot_curr
            Eigen::VectorXd q_curr = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(joint_state.position.data(), joint_state.position.size());
            Eigen::VectorXd qdot_curr = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned> (joint_state.velocity.data(), joint_state.velocity.size());
            // - Desired tau_des, q_des, qdot_des for index i
            Eigen::VectorXd tau_des = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(goal_points[i].effort.data(), goal_points[i].effort.size());
            Eigen::VectorXd q_des = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned> (goal_points[i].positions.data(), goal_points[i].positions.size());
            Eigen::VectorXd qdot_des = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned> (goal_points[i].velocities.data(), goal_points[i].velocities.size());
            //Calculate tau to correct the desired tau
            Eigen::VectorXd tau_added = Kp.cwiseProduct(q_des - q_curr) + Kd.cwiseProduct(qdot_des - qdot_curr);
            //Calculate tau to send and it to point_command
            Eigen::VectorXd tau_total = tau_des + tau_added;
            std::vector<double> tau_total_vec(tau_total.data(), tau_total.data() + tau_total.size());
            trajectory_msgs::JointTrajectoryPoint point_command;
            point_command.positions = goal_points[i].positions;
            point_command.velocities = goal_points[i].velocities;
            point_command.accelerations = goal_points[i].accelerations;
            point_command.effort = tau_total_vec;
            point_command.time_from_start = ros::Duration(time_from_start);
            //Save point_commanded and joint_state into vectors
            trajectory_commanded.points.push_back(point_command);
            trajectory_joint_state.push_back(joint_state);
            //Command gazebo robot
            iiwa_gazebo_command_pub.publish(point_command);
            //Sleep for a fixed amount of time, in this case we use the sample_time but it doesn't matter if you raise it a bit more/less, as the next chosen index will depend on the amount of time passed since the beginning and the command sent will adjust to it, not depending on the control time
            std::cout << i << std::endl;
            ros::Duration(sample_time).sleep();
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
