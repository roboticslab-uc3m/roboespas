#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <iiwa_command/MoveJTrajAction.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen3/Eigen/Dense>
#include "FRIClient.cpp"


using namespace std;

class FRIMoveJTrajAction
{
    protected:
    //Iiwa command action server variables
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<iiwa_command::MoveJTrajAction> as;
    iiwa_command::MoveJTrajFeedback as_feedback;
    iiwa_command::MoveJTrajResult as_result;
    Eigen::VectorXd q_max;
    Eigen::VectorXd qdot_max;
    Eigen::VectorXd qinc_max;
    sensor_msgs::JointState joint_state;
    FRIClient* friClient;
    double control_step_size;
    double min_inc;
    public:

    FRIMoveJTrajAction(std::string name, FRIClient* friClient_) :
    as(nh, name, boost::bind(&FRIMoveJTrajAction::callback_MoveJTraj, this, _1), false) //Create the action server
    {
        friClient = friClient_;
        as.start();
        std::vector<double> q_max_vec;
        if (!nh.getParam("/iiwa/limits/joint_position", q_max_vec))
        {
            ROS_ERROR("Failed to read '/iiwa/limits/joint_position' on param server");
        }
        q_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_max_vec.data(), q_max_vec.size());
        q_max = q_max*M_PI/180.0;
        vector<double> qdot_max_vec;
        if (!nh.getParam("/iiwa/limits/joint_velocity", qdot_max_vec))
        {
            ROS_ERROR("Failed to read '/iiwa/limits/joint_velocity' on param server");
        }
        qdot_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(qdot_max_vec.data(), qdot_max_vec.size());
        qdot_max = qdot_max*M_PI/180.0;
        if (!nh.getParam("/iiwa_command/min_increment", min_inc))
        {
            ROS_ERROR("Failed to read '/iiwa_command/min_increment' on param server");
        }
        ROS_INFO("Action server %s started", name.c_str());
    }
    void commandJointPosition(double q_command[])
    {
        double q_read[7];
        memcpy(q_read, friClient->q_read, LBRState::NUMBER_OF_JOINTS * sizeof(double));
        double diff[7];
        double steady = true;
        for (int i=0; i<7; i++)
        {
          diff[i] = abs(q_read[i]-q_command[i]);
          if (diff[i]>min_inc)
          {
            steady = false;
          }
        }
        if (!steady)
        {
          memcpy(friClient->q_command, q_command, LBRState::NUMBER_OF_JOINTS * sizeof(double));
        }
        else
        {
          cout << "steady" << endl;
        }
    }
    sensor_msgs::JointState getCurrentJointState()
    {
      double q_read[7];
      double qtorque_read[7];
      ros::Time curr_time_stamp;
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
      return joint_state;
    }
    void callback_MoveJTraj(const iiwa_command::MoveJTrajGoalConstPtr &goal)
    {
        ROS_INFO("MoveJTraj action server started...");
        //Variables returned
        trajectory_msgs::JointTrajectory trajectory_commanded;
        std::vector<sensor_msgs::JointState> trajectory_read;
        trajectory_msgs::JointTrajectory trajectory_desired = goal->trajectory_desired;
        //Calculate qinc_max
        if (!nh.getParam("/iiwa_command/control_step_size", control_step_size))
        {
            ROS_ERROR("Failed to read '/iiwa_command/control_step_size' on param server");
        }
        qinc_max = qdot_max*control_step_size*1.5; //Convert to radians and multiply by control_step_size
        //Check trajectory is not empty
        if (trajectory_desired.points.empty())
        {
            ROS_ERROR("Empty joint trajectory");
            as_result.trajectory_commanded = trajectory_commanded;
            as_result.trajectory_read = trajectory_read;
            as.setSucceeded(as_result);
            return;
        }
        //Transform messages into command messages
        double qtraj_command[trajectory_desired.points.size()][7];
        for (int i=0; i<trajectory_desired.points.size(); i++)
        {
            copy(trajectory_desired.points[i].positions.begin(), trajectory_desired.points[i].positions.end(), qtraj_command[i]);

        }
        //Command trajectory
        ros::Duration sleepDur;
        ros::Time contTime;
        ros::Duration proc_time;
        sensor_msgs::JointState joint_state;
        trajectory_msgs::JointTrajectoryPoint point_command;
        double q_command[7];
        double q_ref[7];
        int id = 0;
        //Prepare first variable to command, first points to save
        copy(trajectory_desired.points[0].positions.begin(), trajectory_desired.points[0].positions.end(), q_command);
        ros::Time tStartTraj = ros::Time::now();
        for (int i=1; i<trajectory_desired.points.size(); i++)
        {
            //Save the joint_state in the almost exact moment of commanding the robot
            ros::Duration stamp = ros::Time::now()-tStartTraj;
            joint_state = getCurrentJointState();
            commandJointPosition(q_command);
            //Save commanded and read positions
            point_command.positions.clear();
            for (int i=0; i<7; i++)
            {
                point_command.positions.push_back(q_command[i]);
            }
            point_command.time_from_start = stamp;
            trajectory_commanded.points.push_back(point_command);
            joint_state.header.stamp = ros::Time(stamp.toSec());
            trajectory_read.push_back(joint_state);
            //Prepare next joint positions
            memcpy(q_ref, q_command, LBRState::NUMBER_OF_JOINTS * sizeof(double)); //Save previously commanded position in q_ref
            copy(trajectory_desired.points[i].positions.begin(), trajectory_desired.points[i].positions.end(), q_command);
            double diff[7];
            for (int j=0; j<7; j++)
            {
                diff[j] = joint_state.position[j]-q_ref[j];
                //Modify q_command with a proportion of diff
                q_command[j] = q_command[j];//-diff[j]*0.5;
            }
            //Wait exactly until next iteration
            contTime = tStartTraj + ros::Duration(control_step_size*(i));
            sleepDur = contTime - ros::Time::now();
            if (sleepDur.toSec()<=0)
            {
              cout << sleepDur << endl;
              ROS_ERROR("Too much calculations inside control loop");
            }
            sleepDur.sleep();
        }
        //Command last position and save it
        commandJointPosition(q_command);
        point_command.positions.clear();
        for (int i=0; i<7; i++)
        {
            point_command.positions.push_back(q_command[i]);
        }
        point_command.time_from_start = ros::Time::now()-tStartTraj;
        trajectory_commanded.points.push_back(point_command);
        //Keep capturing for 0.5 seconds
        int nsamples = 0.5/control_step_size;
        nsamples=std::max(nsamples, 1);
        for (int i=0; i<nsamples; i++)
        {
            joint_state = getCurrentJointState();
            joint_state.header.stamp = ros::Time((ros::Time::now()-tStartTraj).toSec());
            trajectory_read.push_back(joint_state);
            ros::Duration(control_step_size).sleep();
        }
        as_result.trajectory_commanded = trajectory_commanded;
        as_result.trajectory_read = trajectory_read;
        as.setSucceeded(as_result);
        ROS_INFO("MoveJTraj action server finished.");
    }
};
