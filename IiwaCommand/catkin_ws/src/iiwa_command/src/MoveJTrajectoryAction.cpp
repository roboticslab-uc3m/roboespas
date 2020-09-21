#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <iiwa_command/MoveJTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <cmath>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/TimeToDestination.h>

using namespace std;

class MoveJTrajectoryAction
{
    protected:
    ros::NodeHandle nh;
    //Iiwa command action server variables
    actionlib::SimpleActionServer<iiwa_command::MoveJTrajectoryAction> as;
    iiwa_command::MoveJTrajectoryFeedback as_feedback;
    iiwa_command::MoveJTrajectoryResult as_result;
    //Joint state subscriber: Gazebo/FRI
    ros::Subscriber iiwa_state_sub;
    sensor_msgs::JointState joint_state;
    //Joint trajectory point publisher: Gazebo/FRI
    ros::Publisher iiwa_command_pub;
    //Publisher and subscriber for iiwa_stack
    ros::Publisher iiwa_stack_command_position;
    ros::ServiceClient timeToDestClient;

    //Parameters
    double control_step_size;
    std::string robot_mode;
    Eigen::VectorXd qdot_max;
    Eigen::VectorXd q_max;
    double error_joint_position_stop=0.0001;

    public:

    MoveJTrajectoryAction(std::string name) :
    as(nh, name, boost::bind(&MoveJTrajectoryAction::callback_MoveJTrajectory, this, _1), false) //Create the action server
    {
        as.start();
        ROS_INFO("Action server %s started", name.c_str());
        //Read parameters
        if(!nh.getParam("/iiwa_command/robot_mode", robot_mode))
        {
            ROS_ERROR("Failed to read '/iiwa_command/robot_mode' on param server");
        }
        if (!nh.getParam("/iiwa_command/control_step_size", control_step_size))
        {
            ROS_ERROR("Failed to read '/iiwa_command/control_step_size' on param server");
        }
        std::vector<double> q_max_vec;
        if (!nh.getParam("/iiwa/limits/joint_position", q_max_vec))
        {
            ROS_ERROR("Failed to read '/iiwa/limits/joint_position' on param server");
        }
        q_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_max_vec.data(), q_max_vec.size());
        q_max = q_max*M_PI/180.0;
        std::vector<double> qdot_max_vec;
        if (!nh.getParam("/iiwa/limits/joint_velocity", qdot_max_vec))
        {
            ROS_ERROR("Failed to read '/iiwa/limits/joint_velocity' on param server");
        }
        qdot_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(qdot_max_vec.data(), qdot_max_vec.size());
        qdot_max = qdot_max*M_PI/180.0;
        //Initializate topics depending on robot_mode
        if (strcmp(robot_mode.c_str(), "gazebo")==0)
        {
            iiwa_command_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/iiwa_gazebo/joint_command", 1000, false);
            iiwa_state_sub = nh.subscribe("/iiwa_gazebo/joint_state", 1000, &MoveJTrajectoryAction::callback_iiwa_state, this);
        }
        else if (strcmp(robot_mode.c_str(), "fri")==0)
        {
            iiwa_command_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/iiwa_fri/joint_command", 1000, false);
            iiwa_state_sub = nh.subscribe("/iiwa_fri/joint_state", 1000, &MoveJTrajectoryAction::callback_iiwa_state, this);
        }
        else if (strcmp(robot_mode.c_str(), "iiwa_stack")==0)
        {
            iiwa_stack_command_position = nh.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 0);
	          timeToDestClient=nh.serviceClient<iiwa_msgs::TimeToDestination>("/iiwa/state/timeToDestination");
        }
        else
        {
            ROS_ERROR("Not implemented yet");
        }
    }
    void callback_iiwa_state(const sensor_msgs::JointState& iiwa_state_msg)
    {
        joint_state=iiwa_state_msg;
    }
    void callback_MoveJTrajectory(const iiwa_command::MoveJTrajectoryGoalConstPtr &goal)
    {
        ROS_INFO("MoveJTrajectory action server active");
        //Variables returned
        trajectory_msgs::JointTrajectory trajectory_commanded;
        std::vector<sensor_msgs::JointState> trajectory_read;
        //First save in an std::vector
        trajectory_msgs::JointTrajectory trajectory_desired = goal -> trajectory_desired;
        //Check position is not empty
        if (trajectory_desired.points.empty())
        {
            ROS_ERROR("Empty joint trajectory");
            as.setSucceeded(as_result);
            return;
        }
        //Variables returned
        int id = 0;
        //Time variables
        ros::Time tStartTraj = ros::Time::now();
        ros::Time currentTime = tStartTraj;
        while (id < trajectory_desired.points.size())
        {
            //Check which position should be commanded now
            currentTime = ros::Time::now();
            id = (currentTime-tStartTraj).toSec()/control_step_size;
            if (id>=trajectory_desired.points.size())
            {
              break;
            }
            //Prepare variables to command
            std::vector<double> q_next = trajectory_desired.points[id].positions;
            ros::Duration time_from_start = trajectory_desired.points[id].time_from_start;
            trajectory_msgs::JointTrajectoryPoint point_command;
            for (int i=0; i<q_next.size(); i++)
            {
                point_command.positions.push_back(q_next[i]);
            }
            point_command.time_from_start = ros::Time::now()-tStartTraj;
            trajectory_commanded.points.push_back(point_command);
            trajectory_read.push_back(joint_state);
            iiwa_command_pub.publish(point_command);
            //cout << "id: " << id << " q_next: " << q_next[5] << " t: " << time_from_start.toSec() << endl;
            ros::Time contTime = tStartTraj + ros::Duration(control_step_size*(id+1));
            ros::Duration sleepDur = contTime-ros::Time::now();
            sleepDur.sleep();
        }
        //ros::Duration time_passed = ros::Time::now() - tStartTraj;
        //cout << time_passed.toSec() << endl;
        as_result.trajectory_commanded = trajectory_commanded;
        as_result.trajectory_read = trajectory_read;
        /*
        //Limit velocity
        //Read velocity percentage
        double velocity=0.5; //from 0 to 1
        if (!nh.getParam("/iiwa_command/velocity", velocity))
        {
            ROS_ERROR("Failed to read '/iiwa_command/velocity' on param server, using 0.5");
        }
        //If mode is gazebo or fri, the joint_position should be divided in several near joint_positions
        if (strcmp(robot_mode.c_str(), "gazebo")==0 || strcmp(robot_mode.c_str(), "fri")==0)
        {
            //Calculate qdot_max with that percentage
            Eigen::VectorXd qdot = qdot_max * velocity;
        */
        as.setSucceeded(as_result);
        ROS_INFO("MoveJTrajectoryTrajectory action server result sent");

    }
};
