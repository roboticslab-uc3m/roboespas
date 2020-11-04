#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <iiwa_command/MoveJTrajCartVelAction.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen3/Eigen/Dense>
#include "FRIClient.cpp"
#include "IiwaScrewTheory.cpp"


using namespace std;

class FRIMoveJTrajCartVelAction
{
    protected:
    //Iiwa command action server variables
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<iiwa_command::MoveJTrajCartVelAction> as;
    iiwa_command::MoveJTrajCartVelFeedback as_feedback;
    iiwa_command::MoveJTrajCartVelResult as_result;
    Eigen::VectorXd q_max;
    Eigen::VectorXd qdot_max;
    Eigen::VectorXd qinc_max;
    sensor_msgs::JointState joint_state;
    FRIClient* friClient;
    double control_step_size;
    double min_inc;
    double kp;
    public:

    FRIMoveJTrajCartVelAction(std::string name, FRIClient* friClient_) :
    as(nh, name, boost::bind(&FRIMoveJTrajCartVelAction::callback_MoveJTrajCartVel, this, _1), false) //Create the action server
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
        if (!nh.getParam("/iiwa_command/control_cart_vel/kp", kp))
        {
            ROS_ERROR("Failed to read '/iiwa_command/control_cart_vel/kp' on param server");
        }
        ROS_INFO("Action server %s started", name.c_str());
    }
    void commandJointPosition(double q_command[])
    {
        double q_read[7];
        memcpy(q_read, friClient->q_read, LBRState::NUMBER_OF_JOINTS * sizeof(double));
        double diff[7];
        double steady = true;
        min_inc = 0.000000001;
        //TODO: Check inside workspace
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
    geometry_msgs::Twist MultiplyTwist(geometry_msgs::Twist t, double n)
    {
      geometry_msgs::Twist t_out;
      t_out.linear.x = t.linear.x*n;
      t_out.linear.y = t.linear.y*n;
      t_out.linear.z = t.linear.z*n;
      t_out.angular.x = t.angular.x*n;
      t_out.angular.y = t.angular.y*n;
      t_out.angular.z = t.angular.z*n;
      return t_out;
    }
    geometry_msgs::Twist SumTwist(geometry_msgs::Twist t1, geometry_msgs::Twist t2)
    {
      geometry_msgs::Twist t_out;
      t_out.linear.x = t1.linear.x + t2.linear.x;
      t_out.linear.y = t1.linear.y + t2.linear.y;
      t_out.linear.z = t1.linear.z + t2.linear.z;
      t_out.angular.x = t1.angular.x + t2.angular.x;
      t_out.angular.y = t1.angular.y + t2.angular.y;
      t_out.angular.z = t1.angular.z + t2.angular.z;
      return t_out;
    }
    void callback_MoveJTrajCartVel(const iiwa_command::MoveJTrajCartVelGoalConstPtr &goal)
    {
        ROS_INFO("MoveJTrajCartVel action server started...");
        //Variables returned
        trajectory_msgs::JointTrajectory trajectory_commanded;
        std::vector<sensor_msgs::JointState> trajectory_read;
        std::vector<geometry_msgs::TwistStamped> cart_vel_desired = goal->cart_vel_desired;
        std::vector<geometry_msgs::TwistStamped> cart_pos_desired = goal->cart_pos_desired;
        //Check trajectory is not empty
        if (cart_vel_desired.empty() || cart_pos_desired.empty())
        {
            ROS_ERROR("Empty cartesian trajectory");
            as_result.trajectory_commanded = trajectory_commanded;
            as_result.trajectory_read = trajectory_read;
            as.setSucceeded(as_result);
            return;
        }
        //Load kp
        if (!nh.getParam("/iiwa_command/control_cart_vel/kp", kp))
        {
            ROS_ERROR("Failed to read '/iiwa_command/control_cart_vel/kp' on param server");
        }
        //Load control_step_size
        if (!nh.getParam("/iiwa_command/control_step_size", control_step_size))
        {
            ROS_ERROR("Failed to read '/iiwa_command/control_step_size' on param server");
        }
        //Set parameters for IiwaScrewTheory library
        std::vector<double> twists;
        if (!nh.getParam("/iiwa/twists", twists))
        {
            ROS_ERROR("Failed to read '/iiwa/twists' on param server");
        }
        Eigen::MatrixXd IiwaTwists = Eigen::Map<Eigen::Matrix<double, 7, 6>>(twists.data()).transpose();
        std::vector<double> hst0;
        if (!nh.getParam("/iiwa/Hst0", hst0))
        {
            ROS_ERROR("Failed to read '/iiwa/Hst0' on param server");
        }
        Eigen::Matrix4d Hst0 = Eigen::Map<Eigen::Matrix<double, 4,4>>(hst0.data()).transpose();
        std::vector<double> qdotmax;
        if (!nh.getParam("/iiwa/limits/joint_velocity", qdotmax))
        {
          ROS_ERROR("Failed to read '/iiwa/limits/joint_velocity on param server'");
        }
        Eigen::VectorXd qdotMax = Eigen::Map<Eigen::Matrix<double, 1, 7>> (qdotmax.data());
        qdotMax = qdotMax*M_PI/180;
        IiwaScrewTheory::SetParameters(IiwaTwists, Hst0, qdotMax);
        //Command trajectory
        double q_command[7];
        trajectory_msgs::JointTrajectoryPoint point_command;
        ros::Duration tTotal = cart_vel_desired[cart_vel_desired.size()-1].header.stamp - cart_vel_desired[0].header.stamp;
        ros::Time tStart = ros::Time::now();
        ros::Time tEnd = tStart + tTotal;
        /*
        while ((ros::Time::now()-tStart) < tTotal) //ros::Duration(0.5))//
        {
            ros::Time tCurr = ros::Time::now();
            ros::Duration stamp = tCurr-tStart;
            int id = floor(stamp.toSec()/control_step_size);
            if (id>cart_pos_desired.size()-1)
            {
              break;
            }
            geometry_msgs::Twist xdot_ref = cart_vel_desired[id].twist;
            geometry_msgs::Twist x_ref = cart_pos_desired[id].twist;
            joint_state = getCurrentJointState();
            geometry_msgs::Twist x_curr = IiwaScrewTheory::ForwardKinematics(joint_state.position);
            cout << id << endl;
            cout << "x: [" << x_curr.linear.x << "," << x_curr.linear.y << "," << x_curr.linear.z << "," << x_curr.angular.x << "," << x_curr.angular.y << "," << x_curr.angular.z << "]" << endl;
            //geometry_msgs::Twist xdot_ref = IiwaScrewTheory::ScrewA2B_S(cart_pos_desired[id].twist, cart_pos_desired[id+1].twist);
            //xdot_ref = MultiplyTwist(xdot_ref, 1/control_step_size);
            //geometry_msgs::Twist e_x = IiwaScrewTheory::ScrewA2B_S(x_curr, x_ref);
            //e_x = MultiplyTwist(e_x, 1/control_step_size);
            geometry_msgs::Twist xdot_com = xdot_ref; //SumTwist(xdot_ref, e_x);
            cout << "xdot_com: [" << xdot_com.linear.x << "," << xdot_com.linear.y << "," << xdot_com.linear.z << "," << xdot_com.angular.x << "," << xdot_com.angular.y << "," << xdot_com.angular.z << "]" << endl;
        }*/
        joint_state = getCurrentJointState();
        vector<double> q_curr {joint_state.position[0], joint_state.position[1], joint_state.position[2], joint_state.position[3], joint_state.position[4], joint_state.position[5], joint_state.position[6]};
        for (int i=0; i<cart_vel_desired.size(); i++)
        {
          ros::Time tCurr = ros::Time::now();
          ros::Duration stamp = tCurr-tStart;
          geometry_msgs::Twist x_ref = cart_pos_desired[i].twist;
          geometry_msgs::Twist xdot_ref = cart_vel_desired[i].twist;
          joint_state = getCurrentJointState();
          geometry_msgs::Twist x_curr = IiwaScrewTheory::ForwardKinematics(joint_state.position);
          geometry_msgs::Twist e_x = IiwaScrewTheory::ScrewA2B_S(x_curr, x_ref);
          e_x = MultiplyTwist(e_x, 1/control_step_size);
          geometry_msgs::Twist xdot_com = SumTwist(xdot_ref, MultiplyTwist(e_x, kp));
          //cout << "xdot_com: [" << xdot_com.linear.x << "," << xdot_com.linear.y << "," << xdot_com.linear.z << "," << xdot_com.angular.x << "," << xdot_com.angular.y << "," << xdot_com.angular.z << "]" << endl;
          vector<double> qdot = IiwaScrewTheory::InverseDifferentialKinematicsPoint(joint_state.position, xdot_com);
          //cout << "q_curr: " << q_curr[0] << ", " << q_curr[1] << ", " << q_curr[2] << ", " << q_curr[3] << ", " << q_curr[4] << ", " << q_curr[5] << ", " << q_curr[6] << endl;
          //cout << "qdot_com: " << qdot[0] << ", " << qdot[1] << ", " << qdot[2] << ", " << qdot[3] << ", " << qdot[4] << ", " << qdot[5] << ", " << qdot[6] << endl;
          q_command[0] = q_curr[0] + qdot[0]*control_step_size;
          q_command[1] = q_curr[1] + qdot[1]*control_step_size;
          q_command[2] = q_curr[2] + qdot[2]*control_step_size;
          q_command[3] = q_curr[3] + qdot[3]*control_step_size;
          q_command[4] = q_curr[4] + qdot[4]*control_step_size;
          q_command[5] = q_curr[5] + qdot[5]*control_step_size;
          q_command[6] = q_curr[6] + qdot[6]*control_step_size;
          q_curr = {q_command[0], q_command[1], q_command[2], q_command[3], q_command[4], q_command[5], q_command[6]};
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
          //Wait until next iteration
          ros::Time contTime = tStart+ ros::Duration(control_step_size*i);
          ros::Duration tPause = contTime - ros::Time::now();
          //cout << tPause.toSec() << endl;
          if (tPause.toSec()>0)
          {
            tPause.sleep();
          }
          else {
            cout << tPause.toSec() << endl;
          }
        }
        int nsamples = 0.5/control_step_size;
        nsamples=std::max(nsamples, 1);
        for (int i=0; i<nsamples; i++)
        {
            joint_state = getCurrentJointState();
            joint_state.header.stamp = ros::Time((ros::Time::now()-tStart).toSec());
            trajectory_read.push_back(joint_state);
            ros::Duration(control_step_size).sleep();
        }
        as_result.trajectory_commanded = trajectory_commanded;
        as_result.trajectory_read = trajectory_read;
        as.setSucceeded(as_result);
        ROS_INFO("MoveJTrajCartVel action server finished.");
    }
};
