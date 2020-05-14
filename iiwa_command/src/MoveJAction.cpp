#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <iiwa_command/MoveJAction.h>
#include <actionlib/server/simple_action_server.h>
#include <Eigen/Dense>
#include <math.h>
#include <cmath>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/TimeToDestination.h>

using namespace std;

class MoveJAction
{
    protected:
    ros::NodeHandle nh;
    //Iiwa command action server variables
    actionlib::SimpleActionServer<iiwa_command::MoveJAction> as;
    iiwa_command::MoveJFeedback as_feedback;
    iiwa_command::MoveJResult as_result;
    //Iiwa gazebo state subscriber
    ros::Subscriber iiwa_state_sub;
    sensor_msgs::JointState joint_state;
    //Iiwa gazebo/fri command publisher
    ros::Publisher iiwa_command_pub;
    //Iiwa_stack
    ros::Publisher iiwa_command_position_pub;
    ros::ServiceClient timeToDestClient;
    //Parameters
    double control_step_size;
    std::string robot_mode;
    Eigen::VectorXd qdot_max;
    Eigen::VectorXd q_max;
    double error_joint_position_stop=0.0001;

    public:

    MoveJAction(std::string name) :
    as(nh, name, boost::bind(&MoveJAction::callback_MoveJ, this, _1), false) //Create the action server
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
        if (!nh.getParam("/iiwa_limits/joint_position", q_max_vec))
        {
            ROS_ERROR("Failed to read '/iiwa_limits/joint_position' on param server");
        }
        q_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_max_vec.data(), q_max_vec.size());
        q_max = q_max*M_PI/180.0;
        std::vector<double> qdot_max_vec;
        if (!nh.getParam("/iiwa_limits/joint_velocity", qdot_max_vec))
        {
            ROS_ERROR("Failed to read '/iiwa_limits/joint_velocity' on param server");
        }
        qdot_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(qdot_max_vec.data(), qdot_max_vec.size());
        qdot_max = qdot_max*M_PI/180.0;
        //Initializate topics depending on robot_mode
        if (strcmp(robot_mode.c_str(), "gazebo")==0)
        {
            iiwa_command_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/iiwa_gazebo/joint_command", 1000, false);
            iiwa_state_sub = nh.subscribe("/iiwa_gazebo/joint_state", 1000, &MoveJAction::callback_iiwa_gazebo_state, this);
        }
        else if (strcmp(robot_mode.c_str(), "iiwa_stack")==0)
        {
            iiwa_command_position_pub = nh.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 0);
	        timeToDestClient=nh.serviceClient<iiwa_msgs::TimeToDestination>("/iiwa/state/timeToDestination");
        }
        else
        {
            ROS_ERROR("Not implemented yet");
        }        
    }
    void callback_iiwa_gazebo_state(const sensor_msgs::JointState& iiwa_gazebo_state_msg)
    {
        joint_state=iiwa_gazebo_state_msg;
    }
    void callback_MoveJ(const iiwa_command::MoveJGoalConstPtr &goal)
    {   
        ROS_INFO("MoveJ action server active");
        //Variables returned
        trajectory_msgs::JointTrajectory trajectory_commanded;
        std::vector<sensor_msgs::JointState> trajectory_joint_state;
        //First save in an std::vector
        std::vector<double> q_goal_vec=goal->joint_position;
        //Check position is not empty
        if (q_goal_vec.empty())
        {
            ROS_ERROR("Empty joint position");
            as.setSucceeded(as_result);
            return;
        }
        //Transform into Eigen::VectorXd
        Eigen::VectorXd q_goal = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned> (q_goal_vec.data(), q_goal_vec.size());
        //Check position is inside the workspace
        for (int j=0; j<q_goal.size(); j++)
        {
            if (std::abs(q_goal[j]) >= q_max[j])
            {
                ROS_ERROR("Goal joint position outside workspace");
                as_result.trajectory_joint_state=trajectory_joint_state;
                as_result.trajectory_commanded=trajectory_commanded;
                as.setSucceeded(as_result);
                return;
            }
        }
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
            //Calculate maximum joint position increment for each joint, taking into account the control step size and the current maximum velocity
            // qdot = incq/t -> incq= qdot*t 
            Eigen::VectorXd qinc_max = qdot*control_step_size;        
            ros::Time tStartTraj = ros::Time::now();
            bool cont=true;
            while (cont)
            {
                sensor_msgs::JointState freezed_joint_state = joint_state;
                //Get current position;
                Eigen::VectorXd q_curr = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(freezed_joint_state.position.data(), freezed_joint_state.position.size());
                //Calculate the difference in radians        
                Eigen::VectorXd q_diff = q_goal-q_curr;
                //Check if it is far from the goal position
                Eigen::VectorXd ratio_far = q_diff.array()/qinc_max.array();
                double ratio_farest = ratio_far.array().abs().maxCoeff();
                if (ratio_farest>=1)
                {
                    q_diff = q_diff.array()/ratio_farest;
                } //else stay as it is

                //Get the next joint position
                Eigen::VectorXd q_next = q_curr + q_diff;
                //Check its inside the workspace
                for (int j=0; j<q_next.size(); j++)
                {
                    if (std::abs(q_next[j]) >= q_max[j])
                    {
                        ROS_ERROR("Intermediate joint position outside workspace");
                        as_result.trajectory_joint_state=trajectory_joint_state;
                        as_result.trajectory_commanded=trajectory_commanded;
                        as.setSucceeded(as_result);
                        return;
                    }
                }
                //Prepare variables to command
                trajectory_msgs::JointTrajectoryPoint point_command;
                for (int i=0; i<q_next.size(); i++)
                {
                    point_command.positions.push_back(q_next[i]);
                }
                ros::Duration time_from_start = ros::Time::now()-tStartTraj;
                point_command.time_from_start = time_from_start;

                //Save point_commanded and joint_state into vectors
                trajectory_commanded.points.push_back(point_command);
                trajectory_joint_state.push_back(freezed_joint_state);
                as_feedback.joint_state = freezed_joint_state;
                as_feedback.point_commanded = point_command;
                as_feedback.time_from_start=time_from_start.toSec();
                as.publishFeedback(as_feedback);
                //Command it
                iiwa_command_pub.publish(point_command);
                cont = q_diff.array().abs().maxCoeff() > error_joint_position_stop;
                ros::Duration(control_step_size).sleep();
            }
            as_result.trajectory_joint_state=trajectory_joint_state;
            as_result.trajectory_commanded=trajectory_commanded;
        }
        else if(strcmp(robot_mode.c_str(), "iiwa_stack")==0)
        {
            //Create the message to send and fill it
	        iiwa_msgs::JointPosition jPos;
	        jPos.position.a1=q_goal[0];
	        jPos.position.a2=q_goal[1];
	        jPos.position.a3=q_goal[2];
	        jPos.position.a4=q_goal[3];
	        jPos.position.a5=q_goal[4];
	        jPos.position.a6=q_goal[5];
	        jPos.position.a7=q_goal[6];
	        iiwa_command_position_pub.publish(jPos);
	        iiwa_msgs::TimeToDestination timeToDestService;
	        //Wait until the remaining time to reach that joint position is under 0.1 seconds and higher than -0.1 to work around the problem that makes the service timeToDestination return a really low number the first few times called. The loop is exited if remainingTime=-999, which means there's an error, or the remainingTime is around 0.
	        /*float remainingTime=1;
	        while (remainingTime<-0.1 || remainingTime>0.1)
	        {
		        //Check the remaining time every iteration
		        timeToDestClient.call(timeToDestService);
		        remainingTime=timeToDestService.response.remaining_time;
		        ros::Duration(0.05).sleep();
		        if (remainingTime==-999)
			        break;
	        }
	        //If there was an error, return false
	        if (remainingTime==-999)
	        {
		        res.success=false;
		        ROS_INFO("Error");
	        }
	        else
	        {
		        res.success=true;
	        }
            */ //TODO:Uncomment when connected to iiwa_stack to check it works
        }
        as.setSucceeded(as_result);
        ROS_INFO("MoveJ action server result sent");
    }
};

