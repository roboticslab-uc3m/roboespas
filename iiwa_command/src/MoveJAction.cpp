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
    //Iiwa gazebo command publisher
    ros::Publisher iiwa_command_pub;
    //Parameters
    double max_joint_position_inc;
    double control_step_size;
    std::string robot_mode;
    Eigen::VectorXd max_vel;
    Eigen::VectorXd max_pos;

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
        std::vector<double> max_pos_vec;
        if (!nh.getParam("/iiwa_limits/joint_position", max_pos_vec))
        {
            ROS_ERROR("Failed to read '/iiwa_limits/joint_position' on param server");
        }
        max_pos = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(max_pos_vec.data(), max_pos_vec.size());

        std::vector<double> max_vel_vec;
        if (!nh.getParam("/iiwa_limits/joint_velocity", max_vel_vec))
        {
            ROS_ERROR("Failed to read '/iiwa_limits/joint_velocity' on param server");
        }
        max_vel = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(max_vel_vec.data(), max_vel_vec.size());
        max_vel = max_vel*M_PI/180.0;
        //Initializate topics depending on robot_mode
        if (strcmp(robot_mode.c_str(), "gazebo")==0)
        {
            iiwa_command_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/iiwa_gazebo/joint_command", 1000, false);
            iiwa_state_sub = nh.subscribe("/iiwa_gazebo/joint_state", 1000, &MoveJAction::callback_iiwa_gazebo_state, this);
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
        //Check position is not empty
        if (goal->joint_position.empty())
        {
            ROS_ERROR("Empty joint position");
            as.setSucceeded(as_result);
            return;
        }
        //Check position is inside the workspace
        //TODO
        //Build trajectory to send
        //Read percentage velocity from parameter server
        double velocity=0.5; //from 0 to 1
        if (!nh.getParam("/iiwa_command/velocity", velocity))
        {
            ROS_ERROR("Failed to read '/iiwa_command/velocity' on param server, using 0.5");
        }
        //Calculate joint_velocity for each joint using the velocity percentage and the maximum velocity
        std::cout << velocity << std::endl;
        Eigen::VectorXd qdot = velocity*max_vel;

        //First save in an Eigen vector q_goal and q_curr
        std::vector<double> q_goal_vec=goal->joint_position;
        Eigen::VectorXd q_goal = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned> (q_goal_vec.data(), q_goal_vec.size());
        Eigen::VectorXd q_curr = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(joint_state.position.data(), joint_state.position.size());
        //Calculate the difference in radians
        Eigen::VectorXd q_inc = q_goal-q_curr;
        //Calculate the minimum time to reach that point given      
        Eigen::VectorXd times = q_inc.array()/qdot.array();
        //Get maximum time, which will be the one used
        double time_used = times.maxCoeff();
        std::cout << "time: " << time_used << std::endl;
        //Check maximum increment in radians for joint_position
        if (!nh.getParam("/iiwa_limits/joint_position_inc", max_joint_position_inc))
        {
            ROS_ERROR("Failed to read '/iiwa_limits/joint_position_inc' on param server");
        }
        //Get maximum sample step size for this max increment in radians for joint position
        double max_sample_step_size = max_joint_position_inc * time_used / q_inc.maxCoeff();

        as.setSucceeded(as_result);
        /*
        //Variables used
        std::vector<trajectory_msgs::JointTrajectoryPoint> goal_points = goal->trajectory_desired.points;

        //Variables returned
        trajectory_msgs::JointTrajectory trajectory_commanded;
        std::vector<sensor_msgs::JointState> trajectory_joint_state;

        //Initialize the index that will contain the current index of the trajectory depending on the time that has passed
        int i=0;

        //Get the time at which the trajectory starts being sent
        ros::Time tStartTraj = ros::Time::now();
        while (i<goal_points.size())
        {
            //read_joint_state may change during this iteration, save it in a different variable to fix it
            sensor_msgs::JointState joint_state=joint_state;
            double time_from_start=(ros::Time::now()-tStartTraj).toSec(); 
            //Get the index in the goal_points vector corresponding to the current time
            i = time_from_start/control_step_size;
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
            //Sleep for a fixed amount of time, in this case we use the control_step_size but it doesn't matter if you raise it a bit more/less, as the next chosen index will depend on the amount of time passed since the beginning and the command sent will adjust to it, not depending on the control time
            as_feedback.joint_state = joint_state;
            as_feedback.point_commanded = point_command;
            as_feedback.time_from_start=time_from_start;
            as.publishFeedback(as_feedback);
            ros::Duration(control_step_size).sleep();
        }
        as_result.trajectory_joint_state=trajectory_joint_state;
        as_result.trajectory_commanded=trajectory_commanded;
        as.setSucceeded(as_result);
        ROS_INFO("IiwaCommand action server result sent");*/
    }
};

