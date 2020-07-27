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

class MoveLAction
{
    protected:
    ros::NodeHandle nh;
    //Iiwa command action server variables
    actionlib::SimpleActionServer<iiwa_command::MoveLAction> as;
    iiwa_command::MoveLFeedback as_feedback;
    iiwa_command::MoveLResult as_result;
    //Iiwa gazebo state subscriber
    ros::Subscriber iiwa_state_sub;
    sensor_msgs::JointState joint_state;
    //Iiwa gazebo/fri command publisher
    ros::Publisher iiwa_command_pub;
    //Parameters
    double control_step_size;
    string robot_mode;
    VectorXd qdot_max;
    VectorXd q_max;
    double error_cartesian_position_stop=0.001;
    double idk_error_factor=0;

    public:

    MoveLAction(string name) :
    as(nh, name, boost::bind(&MoveLAction::callback_MoveL, this, _1), false) //Create the action server
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
            iiwa_state_sub = nh.subscribe("/iiwa_gazebo/joint_state", 1000, &MoveLAction::callback_iiwa_state, this);
        }
        else if (strcmp(robot_mode.c_str(), "fri")==0)
        {
            iiwa_command_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/iiwa_fri/joint_command", 1000, false);
            iiwa_state_sub = nh.subscribe("/iiwa_fri/joint_state", 1000, &MoveLAction::callback_iiwa_state, this);
        }
        else
        {
            ROS_ERROR("Not implemented yet");
        }
    }
    void callback_iiwa_state(const sensor_msgs::JointState& iiwa_gazebo_state_msg)
    {
        joint_state=iiwa_gazebo_state_msg;
    }
    void callback_MoveL(const iiwa_command::MoveLGoalConstPtr &goal)
    {
        ROS_INFO("MoveL action server active");
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
        vector<double> x_goal_vec=goal->cartesian_position;
        //Check position is not empty
        if (x_goal_vec.empty())
        {
            ROS_ERROR("Empty cartesian position");
            as.setSucceeded(as_result);
            return;
        }
        //Transform into VectorXd
        VectorXd x_goal = Map<VectorXd, Unaligned> (x_goal_vec.data(), x_goal_vec.size());
        //TODO: Check position is inside the cartesian workspace
        /* for (int j=0; j<x_goal.size(); j++)
        {
            if (abs(x_goal[j]) >= x_max[j]) //*x_max doesn't exist yet
            {
                ROS_ERROR("Goal cartesian position outside workspace");
                as_result.trajectory_read=trajectory_read;
                as_result.trajectory_commanded=trajectory_commanded;
                as.setSucceeded(as_result);
                return;
            }
        }*/

        //If mode is gazebo or fri, the joint_position should be divided in several near joint_positions
        if (strcmp(robot_mode.c_str(), "gazebo")==0 || strcmp(robot_mode.c_str(), "fri")==0)
        {
            //Initialize joint and cartesian positions and velocities
            VectorXd q_curr = Map<VectorXd, Unaligned>(joint_state.position.data(), joint_state.position.size());
            VectorXd q_ini = q_curr;
            VectorXd qdot_comm = VectorXd::Zero(7);
            IiwaTrajectory traj_theory(0, control_step_size);
            bool possible = IiwaTrajectoryGeneration::TrapezoidalVelocityProfileTrajectory(q_ini, x_goal, control_step_size, velocity, 0.5, q_max, qdot_max, traj_theory);
            if (!possible)
            {
                ROS_ERROR ("Trajectory out of workspace");
                as_result.trajectory_read=trajectory_read;
                as_result.trajectory_commanded=trajectory_commanded;
                as_result.x_theory = x_theory;
                as_result.xdot_theory = xdot_theory;
                as.setSucceeded(as_result);
                return;
            }
            //Fill theory traj_theory
            for (unsigned int i=0; i<traj_theory.npoints; i++)
            {
                geometry_msgs::TwistStamped x_i;
                geometry_msgs::TwistStamped xdot_i;
                x_i.header.stamp = ros::Time(traj_theory.t[i]);
                xdot_i.header.stamp = ros::Time(traj_theory.t[i]);
                x_i.twist.linear.x = traj_theory.x.col(i)[0];
                x_i.twist.linear.y = traj_theory.x.col(i)[1];
                x_i.twist.linear.z = traj_theory.x.col(i)[2];
                x_i.twist.angular.x = traj_theory.x.col(i)[3];
                x_i.twist.angular.y = traj_theory.x.col(i)[4];
                x_i.twist.angular.z = traj_theory.x.col(i)[5];
                xdot_i.twist.linear.x = traj_theory.xdot.col(i)[0];
                xdot_i.twist.linear.y = traj_theory.xdot.col(i)[1];
                xdot_i.twist.linear.z = traj_theory.xdot.col(i)[2];
                xdot_i.twist.angular.x = traj_theory.xdot.col(i)[3];
                xdot_i.twist.angular.y = traj_theory.xdot.col(i)[4];
                xdot_i.twist.angular.z = traj_theory.xdot.col(i)[5];
                x_theory.push_back(x_i);
                xdot_theory.push_back(xdot_i);
            }

            bool cont=true;
            int id_output = 0;
            ros::Time tStartTraj = ros::Time::now();
            ros::Duration timeFromStart = ros::Duration(0);
            VectorXd q_comm;
            VectorXd q_exp = q_curr;
            while (cont)
            {
                //Freeze the current joint state
                sensor_msgs::JointState freezed_joint_state = joint_state;
                //Get the id of traj_theory for the current time from start
                timeFromStart = ros::Time::now()-tStartTraj;
                int id_curr = round(timeFromStart.toSec()/control_step_size);
                //Fill q_exp and q_curr if not the first iter
                //(for first iter q_curr is already calculated and q_comm does not exist yet)
                if (id_output != 0)
                {
                    q_exp = q_comm;
                    q_curr = Map<VectorXd, Unaligned>(freezed_joint_state.position.data(), freezed_joint_state.position.size()); //q_exp; //
                }
                //Break the while if id_curr>size(traj_theory)
                if (id_curr >= traj_theory.x.cols())
                {
                    //Last iteration, fill last points
                    cout << "last iter " << endl;
                    id_curr = traj_theory.x.cols() -1;
                    //break;
                }
                //Current cartesian position, and expected cartesian position and velocity
                VectorXd x_exp = traj_theory.x.col(id_curr);
                VectorXd xdot_exp = traj_theory.xdot.col(id_curr);
                VectorXd x_curr = IiwaScrewTheory::ForwardKinematics(q_curr);
                //Cartesian velocity to reach x_exp from x_curr in control_step_size
                VectorXd xdot_err = IiwaScrewTheory::ScrewA2B_A(x_curr, x_exp)/control_step_size;
                //Transform into the S frames both xdots, applied on different points
                VectorXd xdot_exp_S = IiwaScrewTheory::TransformScrew_A2S(xdot_exp, x_exp);
                VectorXd xdot_err_S = IiwaScrewTheory::TransformScrew_A2S(xdot_err, x_curr);
                //The desired cartesian velocity will be a mix of both
                VectorXd xdot_S = xdot_exp_S + xdot_err_S*idk_error_factor;
                qdot_comm = IiwaScrewTheory::InverseDifferentialKinematicsPoint(q_curr, xdot_S);
                if ((qdot_comm.array()>qdot_max.array()).any())
                {
                    VectorXd far_ratio = (qdot_comm.cwiseAbs().array()/qdot_max.array()).matrix();
                    //cout << "far ratio: " << far_ratio.transpose() << endl;
                    double farest_ratio = far_ratio.maxCoeff();
                    //cout << "farest: " << farest_ratio.transpose() << endl;
                    //cout << "prev qdot_comm: " << qdot_comm.transpose() << endl;
                    if (farest_ratio>1)
                    {
                        qdot_comm = (qdot_comm.array()/farest_ratio).matrix();
                    }
                    cout << "new qdot_comm: " << qdot_comm.transpose() << endl;
                    cout << "qinc: " << (qdot_comm*control_step_size).transpose() << endl;
                    ROS_ERROR("Limited joint velocity");
                }
                q_comm = q_curr + qdot_comm*control_step_size;
                //Check if q_comm is inside the workspace
                for (int j=0; j<q_comm.size(); j++)
                {
                    if (abs(q_comm[j]) >= q_max[j])
                    {
                        ROS_ERROR("Intermediate joint position outside workspace");
                        as_result.trajectory_read=trajectory_read;
                        as_result.trajectory_commanded=trajectory_commanded;
                        as_result.x_theory = x_theory;
                        as_result.xdot_theory = xdot_theory;
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
                cont = (x_goal-x_curr).array().abs().maxCoeff() > error_cartesian_position_stop;
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
        as_result.x_theory = x_theory;
        as_result.xdot_theory = xdot_theory;
        as.setSucceeded(as_result);
        ROS_INFO("MoveL action server result sent");
    }
};
