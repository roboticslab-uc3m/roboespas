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
#include <iiwa_command/MoveLTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <Eigen/Dense>
#include <math.h>
#include <cmath>
#include "IiwaTrajectoryGeneration.cpp"
#include "IiwaScrewTheory.cpp"

using namespace Eigen;
using namespace std;

class MoveLTrajectoryAction
{
    protected:
    ros::NodeHandle nh;
    //Iiwa command action server variables
    actionlib::SimpleActionServer<iiwa_command::MoveLTrajectoryAction> as;
    iiwa_command::MoveLTrajectoryFeedback as_feedback;
    iiwa_command::MoveLTrajectoryResult as_result;
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
    string robot_mode;
    VectorXd qdot_max;
    VectorXd q_max;
    double error_cartesian_position_stop=0.001;
    double idk_error_factor=0;

    public:

    MoveLTrajectoryAction(string name) :
    as(nh, name, boost::bind(&MoveLTrajectoryAction::callback_MoveLTrajectory, this, _1), false) //Create the action server
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
        vector<double> q_max_vec;
        if (!nh.getParam("/iiwa/limits/joint_position", q_max_vec))
        {
            ROS_ERROR("Failed to read '/iiwa/limits/joint_position' on param server");
        }
        q_max = Map<VectorXd, Unaligned>(q_max_vec.data(), q_max_vec.size());
        q_max = q_max*M_PI/180.0;
        vector<double> qdot_max_vec;
        if (!nh.getParam("/iiwa/limits/joint_velocity", qdot_max_vec))
        {
            ROS_ERROR("Failed to read '/iiwa/limits/joint_velocity' on param server");
        }
        qdot_max = Map<VectorXd, Unaligned>(qdot_max_vec.data(), qdot_max_vec.size());
        qdot_max = qdot_max*M_PI/180.0;
        if (!nh.getParam("/iiwa_command/idk_error_factor", idk_error_factor))
        {
            ROS_ERROR("Failed to read '/iiwa_command/idk_error_factor' on param server");
        }
        //Initializate topics depending on robot_mode
        if (strcmp(robot_mode.c_str(), "gazebo")==0)
        {
            iiwa_command_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/iiwa_gazebo/joint_command", 1000, false);
            iiwa_state_sub = nh.subscribe("/iiwa_gazebo/joint_state", 1000, &MoveLTrajectoryAction::callback_iiwa_gazebo_state, this);
        }
        else if (strcmp(robot_mode.c_str(), "iiwa_stack")==0)
        {
            //TODO
            ROS_ERROR("Not implemented yet");
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
    void callback_MoveLTrajectory(const iiwa_command::MoveLTrajectoryGoalConstPtr &goal)
    {
        ROS_INFO("MoveLTrajectory  action server active");
        //Read velocity percentage
        double velocity=0.5; //from 0 to 1
        if (!nh.getParam("/iiwa_command/velocity", velocity))
        {
            ROS_ERROR("Failed to read '/iiwa_command/velocity' on param server, using 0.5");
        }
        //Read control_step_size
        if (!nh.getParam("/iiwa_command/control_step_size", control_step_size))
        {
            ROS_ERROR("Failed to read '/iiwa_command/control_step_size' on param server.");
        }
        //Variables returned
        trajectory_msgs::JointTrajectory trajectory_commanded;
        vector<sensor_msgs::JointState> trajectory_read;
        vector<geometry_msgs::TwistStamped> xdot_theory;
        vector<geometry_msgs::TwistStamped> x_theory;
        //First save in an vector
        vector<geometry_msgs::TwistStamped> trajectory_goal = goal->trajectory_goal;
        vector<double> q_ini_vec=goal->q_ini;
        //Check position is not empty
        if (trajectory_goal.empty())
        {
            ROS_ERROR("Empty trajectory_goal");
            as.setSucceeded(as_result);
            return;
        }
        if (q_ini_vec.empty())
        {
            ROS_ERROR("Empty initial joint position");
            as.setSucceeded(as_result);
            return;
        }
        cout << "traj_goal size: " << trajectory_goal.size() << endl;
        //Transform into MatrixXd
        MatrixXd traj_goal = MatrixXd::Zero(6, trajectory_goal.size());
        for (unsigned int i=0; i<trajectory_goal.size(); i++)
        {
            VectorXd x_i(6);
            x_i[0] = trajectory_goal[i].twist.linear.x;
            x_i[1] = trajectory_goal[i].twist.linear.y;
            x_i[2] = trajectory_goal[i].twist.linear.z;
            x_i[3] = trajectory_goal[i].twist.angular.x;
            x_i[4] = trajectory_goal[i].twist.angular.y;
            x_i[5] = trajectory_goal[i].twist.angular.z;
            traj_goal.col(i) = x_i;
            cout << "x_i: " << x_i  << endl;
        }

        //TODO: Check position is inside the cartesian workspace
        /*
        for (int i=0; i<traj_goal.cols(); i++)
        {
            for (int j=0; j<traj_goal.rows(); j++)
            {
                VectorXd x_goal = traj_goal.col(j);
                if (abs(x_goal[j]) >= x_max[j]) //*x_max doesn't exist yet
                {
                    ROS_ERROR("Goal cartesian position outside workspace");
                    as_result.trajectory_read=trajectory_read;
                    as_result.trajectory_commanded=trajectory_commanded;
                    as.setSucceeded(as_result);
                    return;
                }
            }
        }*/

        //If mode is gazebo or fri, the joint_position should be divided in several near joint_positions
        if (strcmp(robot_mode.c_str(), "gazebo")==0 || strcmp(robot_mode.c_str(), "fri")==0)
        {
            //Initialize joint and cartesian positions and velocities
            VectorXd q_curr = Map<VectorXd, Unaligned>(joint_state.position.data(), joint_state.position.size());
            VectorXd q_ini = Map<VectorXd, Unaligned>(q_ini_vec.data(), q_ini_vec.size());
            ros::Duration(1).sleep();
            VectorXd qdot_needed = (q_curr-q_ini)/control_step_size;
            ros::Duration(1).sleep();
            if ((qdot_needed.array().abs() > qdot_max.array()).any())
            {
                ROS_ERROR("First move the robot to q_ini, then resend trajectory");
                as_result.trajectory_read = trajectory_read;
                as_result.trajectory_commanded = trajectory_commanded;
                as.setSucceeded(as_result);
                return;
            }
            //Move to q_ini
            trajectory_msgs::JointTrajectoryPoint point_ini;
            for (int i=0; i<q_ini.size(); i++)
            {
                point_ini.positions.push_back(q_ini[i]);
                point_ini.velocities.push_back(0.0);
            }
            iiwa_command_pub.publish(point_ini);

            //Follow the rest of the trajectory;

            bool cont=true;
            int id_output = 0;
            VectorXd qdot_comm = VectorXd::Zero(7);
            VectorXd q_comm;
            VectorXd q_exp = q_curr;
            VectorXd x_goal = traj_goal.col(traj_goal.cols()-1);
            ros::Time tStartTraj = ros::Time::now();
            ros::Duration timeFromStart = ros::Duration(0);
            while (cont)
            {
                //Freeze the current joint state
                sensor_msgs::JointState freezed_joint_state = joint_state;
                //Get the id of traj_goal for the current time from start
                timeFromStart = ros::Time::now()-tStartTraj;
                int id_curr = round(timeFromStart.toSec()/control_step_size);
                //Fill q_exp and q_curr if not the first iter
                //(for first iter q_curr is already calculated and q_comm does not exist yet)
                if (id_output != 0)
                {
                    q_exp = q_comm;
                    q_curr = Map<VectorXd, Unaligned>(freezed_joint_state.position.data(), freezed_joint_state.position.size()); //q_exp; //
                }
                //Break the while if id_curr>size(traj_goal)
                if (id_curr >= traj_goal.cols())
                {
                    //Last iteration, fill last points
                    break;
                }
                //Current cartesian position, and expected cartesian position and velocity
                VectorXd x_exp = traj_goal.col(id_curr);
                VectorXd xdot_exp(6);
                if (id_curr==0)
                {
                    VectorXd xdot_next = (traj_goal.col(id_curr+1) - traj_goal.col(id_curr))/control_step_size;
                    xdot_exp = xdot_next;
                }
                else if (id_curr==traj_goal.cols()-1)
                {
                    VectorXd xdot_prev = (traj_goal.col(id_curr) - traj_goal.col(id_curr-1))/control_step_size;
                    xdot_exp = xdot_prev;
                }
                else
                {
                    VectorXd xdot_prev = (traj_goal.col(id_curr) - traj_goal.col(id_curr-1))/control_step_size;
                    VectorXd xdot_next = (traj_goal.col(id_curr+1) - traj_goal.col(id_curr))/control_step_size;
                    xdot_exp = (xdot_prev+xdot_next)/2;
                }
                VectorXd x_curr = IiwaScrewTheory::ForwardKinematics(q_curr);
                //Cartesian velocity to reach x_exp from x_curr in control_step_size
                VectorXd xdot_err = IiwaScrewTheory::ScrewA2B_A(x_curr, x_exp)/control_step_size;
                //Transform into the S frames both xdots, applied on different points
                VectorXd xdot_exp_S = IiwaScrewTheory::TransformScrew_A2S(xdot_exp, x_exp);
                VectorXd xdot_err_S = IiwaScrewTheory::TransformScrew_A2S(xdot_err, x_curr);
                //The desired cartesian velocity will be a mix of both
                VectorXd xdot_S = xdot_exp_S + xdot_err_S*idk_error_factor;
                qdot_comm = IiwaScrewTheory::InverseDifferentialKinematicsPoint(q_curr, xdot_S);
                //TODO: Check qdot max
                q_comm = q_curr + qdot_comm*control_step_size;
                //Check if q_comm is inside the workspace
                for (int j=0; j<q_comm.size(); j++)
                {
                    if (abs(q_comm[j]) >= q_max[j])
                    {
                        ROS_ERROR("Intermediate joint position outside workspace");
                        as_result.trajectory_read=trajectory_read;
                        as_result.trajectory_commanded=trajectory_commanded;
                        as.setSucceeded(as_result);
                        return;
                    }
                }
                //Prepare variables to command
                trajectory_msgs::JointTrajectoryPoint point_command;
                for (int i=0; i<q_comm.size(); i++)
                {
                    point_command.positions.push_back(q_comm[i]);
                    point_command.velocities.push_back(qdot_comm[i]);
                }
                point_command.time_from_start = timeFromStart;
                //Save point_commanded and joint_state into vectors
                trajectory_commanded.points.push_back(point_command);
                trajectory_read.push_back(freezed_joint_state);
                as_feedback.joint_state = freezed_joint_state;
                as_feedback.point_commanded = point_command;
                as_feedback.time_from_start = timeFromStart.toSec();
                as.publishFeedback(as_feedback);
                //Command it
                iiwa_command_pub.publish(point_command);
                //Check if x_goal is near x_goal;
                VectorXd x_diff = x_goal-x_curr;
                cont = x_diff.array().abs().maxCoeff() > error_cartesian_position_stop;
                ros::Duration(control_step_size).sleep();
                id_output++;
            }
        }
        else if(strcmp(robot_mode.c_str(), "iiwa_stack")==0)
        {
            //TODO:Check iiwa_stack topic to publish a cartesian position
        }
        //Fill result
        as_result.trajectory_read=trajectory_read;
        as_result.trajectory_commanded=trajectory_commanded;
        as.setSucceeded(as_result);
        ROS_INFO("MoveL action server result sent");
        return;
    }
};
