#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <iiwa_command/MoveJVelTrajAction.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen3/Eigen/Dense>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointTorque.h>
#include "IiwaScrewTheory.cpp"
#include "geometry_msgs/TwistStamped.h"
#include <iiwa_command/MoveJTrajCartVelAction.h>


using namespace std;

class StackMoveJVelTrajCartVelAction
{
    protected:
    //Iiwa command action server variables
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<iiwa_command::MoveJVelTrajAction> as;
    iiwa_command::MoveJVelTrajFeedback as_feedback;
    iiwa_command::MoveJVelTrajResult as_result;
    Eigen::VectorXd q_max;
    ros::Publisher qdot_pub;
    ros::ServiceClient timeToDestClient;
    ros::Subscriber q_sub;
    ros::Subscriber qdot_sub;
    ros::Subscriber qtorque_sub;
    sensor_msgs::JointState joint_state;
    double kp;
    public:

    StackMoveJVelTrajCartVelAction(std::string name) :
    as(nh, name, boost::bind(&StackMoveJVelTrajCartVelAction::callback_MoveJVelTraj, this, _1), false) //Create the action server
    {
        as.start();
        std::vector<double> q_max_vec;
        if (!nh.getParam("/iiwa/limits/joint_position", q_max_vec))
        {
            ROS_ERROR("Failed to read '/iiwa/limits/joint_position' on param server");
        }
        q_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_max_vec.data(), q_max_vec.size());
        q_max = q_max*M_PI/180.0;
        qdot_pub = nh.advertise<iiwa_msgs::JointVelocity>("/iiwa/command/JointVelocity", 0);
        q_sub = nh.subscribe("/iiwa/state/JointPosition", 1000, &StackMoveJVelTrajCartVelAction::callback_qSub, this);
        qdot_sub = nh.subscribe("/iiwa/state/JointVelocity", 1000, &StackMoveJVelTrajCartVelAction::callback_qdotSub, this);
        qtorque_sub = nh.subscribe("/iiwa/state/JointTorque", 1000, &StackMoveJVelTrajCartVelAction::callback_qtorqueSub, this);
        ROS_INFO("Action server %s started", name.c_str());
    }
    void callback_qSub(const iiwa_msgs::JointPosition& q_msg)
    {
        //Update joint_state
        sensor_msgs::JointState js = joint_state;
        js.position.clear();
        js.position.push_back(q_msg.position.a1);
        js.position.push_back(q_msg.position.a2);
        js.position.push_back(q_msg.position.a3);
        js.position.push_back(q_msg.position.a4);
        js.position.push_back(q_msg.position.a5);
        js.position.push_back(q_msg.position.a6);
        js.position.push_back(q_msg.position.a7);
        //Update stamp
        js.header=q_msg.header;
        joint_state = js;
    }
    void callback_qdotSub(const iiwa_msgs::JointVelocity& qdot_msg)
    {
        //Update velocity
        sensor_msgs::JointState js = joint_state;
        js.velocity.clear();
        js.velocity.push_back(qdot_msg.velocity.a1);
        js.velocity.push_back(qdot_msg.velocity.a2);
        js.velocity.push_back(qdot_msg.velocity.a3);
        js.velocity.push_back(qdot_msg.velocity.a4);
        js.velocity.push_back(qdot_msg.velocity.a5);
        js.velocity.push_back(qdot_msg.velocity.a6);
        js.velocity.push_back(qdot_msg.velocity.a7);
        joint_state = js;
        //Do not update stamp because the stamp received here is wrong
    }
    void callback_qtorqueSub(const iiwa_msgs::JointTorque& qtorque_msg)
    {
        //Update torque
        sensor_msgs::JointState js = joint_state;
        js.effort.clear();
        js.effort.push_back(qtorque_msg.torque.a1);
        js.effort.push_back(qtorque_msg.torque.a2);
        js.effort.push_back(qtorque_msg.torque.a3);
        js.effort.push_back(qtorque_msg.torque.a4);
        js.effort.push_back(qtorque_msg.torque.a5);
        js.effort.push_back(qtorque_msg.torque.a6);
        js.effort.push_back(qtorque_msg.torque.a7);
        //Update stamp
        js.header = qtorque_msg.header;
        joint_state=js;
    }
    void WakeUpRobot(std::vector<double> direction)
    {
        //First prepare the robot to move by sending a really low velocity -> Theres a problem when commanding some velocity to the iiwa:  when it is not previously moving and some velocity is commanded, if it is not really reall low, the robot accelerates too much and goes far away from the desired position. Sending a low velocity works around this problem.
        double v=0.0000001;
        vector<double> qdot_ini={v, v, v, v, v, v, v};
        iiwa_msgs::JointVelocity qdot_ini_msg;
        qdot_ini_msg.velocity.a1=direction[0]*v;
        qdot_ini_msg.velocity.a2=direction[1]*v;
        qdot_ini_msg.velocity.a3=direction[2]*v;
        qdot_ini_msg.velocity.a4=direction[3]*v;
        qdot_ini_msg.velocity.a5=direction[4]*v;
        qdot_ini_msg.velocity.a6=direction[5]*v;
        qdot_ini_msg.velocity.a7=direction[6]*v;
        qdot_pub.publish(qdot_ini_msg);
        ros::Duration(0.1).sleep();

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
    void commandJointVelocity(vector<double> qd_command)
    {
        iiwa_msgs::JointVelocity qd_msg;
        qd_msg.velocity.a1 = qd_command[0];
        qd_msg.velocity.a2 = qd_command[1];
        qd_msg.velocity.a3 = qd_command[2];
        qd_msg.velocity.a4 = qd_command[3];
        qd_msg.velocity.a5 = qd_command[4];
        qd_msg.velocity.a6 = qd_command[5];
        qd_msg.velocity.a7 = qd_command[6];
        qdot_pub.publish(qd_msg);
    }
    void callback_MoveJVelTraj(const iiwa_command::MoveJTrajCartVelGoalConstPtr &goal)
    {
        ROS_INFO("MoveJVelTraj action server started...");
        //Variables returned
        trajectory_msgs::JointTrajectory trajectory_commanded;
        std::vector<sensor_msgs::JointState> trajectory_read;
        std::vector<geometry_msgs::TwistStamped> cart_vel_desired = goal->cart_vel_desired;
        std::vector<geometry_msgs::TwistStamped> cart_pos_desired = goal->cart_pos_desired;
        //Load kp
        if (!nh.getParam("/iiwa_command/control_cart_vel/kp", kp))
        {
            ROS_ERROR("Failed to read '/iiwa_command/control_cart_vel/kp' on param server");
        }
        //Read control_step_size
        double control_step_size;
        if (!nh.getParam("/iiwa_command/control_step_size", control_step_size))
        {
            ROS_ERROR("Failed to read '/iiwa_command/control_step_size' on param server");
        }
        //Check trajectory is not empty
        if (cart_vel_desired.empty() || cart_pos_desired.empty())
        {
            ROS_ERROR("Empty cartesian trajectory");
            as_result.trajectory_commanded = trajectory_commanded;
            as_result.trajectory_read = trajectory_read;
            as.setSucceeded(as_result);
            return;
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
        double qd_command[7];
        trajectory_msgs::JointTrajectoryPoint point_command;
        ros::Duration tTotal = cart_vel_desired[cart_vel_desired.size()-1].header.stamp - cart_vel_desired[0].header.stamp;
        ros::Time tStart = ros::Time::now();
        ros::Time tEnd = tStart + tTotal;

        //int npoints=trajectory_desired.points.size();
        //
        //joint_state = getCurrentJointState();
        sensor_msgs::JointState js = joint_state;
        vector<double> q_curr {js.position[0], js.position[1], js.position[2], js.position[3], js.position[4], js.position[5], js.position[6]};
        for (int i=0; i<cart_vel_desired.size(); i++)
        {
          ros::Time tCurr = ros::Time::now();
          ros::Duration stamp = tCurr-tStart;
          geometry_msgs::Twist x_ref = cart_pos_desired[i].twist;
          geometry_msgs::Twist xdot_ref = cart_vel_desired[i].twist;
          //joint_state = getCurrentJointState();
          js = joint_state;
          geometry_msgs::Twist x_curr = IiwaScrewTheory::ForwardKinematics(js.position);
          geometry_msgs::Twist e_x = IiwaScrewTheory::ScrewA2B_S(x_curr, x_ref);
          e_x = MultiplyTwist(e_x, 1/control_step_size);
          geometry_msgs::Twist xdot_com = SumTwist(xdot_ref, MultiplyTwist(e_x, kp));
          //cout << "xdot_com: [" << xdot_com.linear.x << "," << xdot_com.linear.y << "," << xdot_com.linear.z << "," << xdot_com.angular.x << "," << xdot_com.angular.y << "," << xdot_com.angular.z << "]" << endl;
          vector<double> qdot = IiwaScrewTheory::InverseDifferentialKinematicsPoint(js.position, xdot_com);
          //cout << "q_curr: " << q_curr[0] << ", " << q_curr[1] << ", " << q_curr[2] << ", " << q_curr[3] << ", " << q_curr[4] << ", " << q_curr[5] << ", " << q_curr[6] << endl;
          //cout << "qdot_com: " << qdot[0] << ", " << qdot[1] << ", " << qdot[2] << ", " << qdot[3] << ", " << qdot[4] << ", " << qdot[5] << ", " << qdot[6] << endl;
          commandJointVelocity(qdot);
          //Save commanded and read positions
          point_command.positions.clear();
          for (int i=0; i<7; i++)
          {
              point_command.positions.push_back(q_curr[i] + qdot[i]*control_step_size);
              q_curr[i] = q_curr[i] + qdot[i]*control_step_size);
          }
          //Save commanded and read positions
          point_command.positions.clear();
          for (int i=0; i<7; i++)
          {
              point_command.positions.push_back(q_command[i]);
          }
          point_command.time_from_start = stamp;
          trajectory_commanded.points.push_back(point_command);
          js.header.stamp = ros::Time(stamp.toSec());
          trajectory_read.push_back(js);
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
            js = joint_state;
            js.header.stamp = ros::Time((ros::Time::now()-tStart).toSec());
            trajectory_read.push_back(js);
            ros::Duration(control_step_size).sleep();
        }
        //


        int nsamples = 0.5/control_step_size;
        nsamples=std::max(nsamples, 1);
        for (int i=0; i<nsamples; i++)
        {
           trajectory_read.push_back(joint_state);
           ros::Duration(control_step_size).sleep();
        }
        as_result.trajectory_commanded = trajectory_desired;
        as_result.trajectory_read = trajectory_read;
        as.setSucceeded(as_result);
        ROS_INFO("MoveJVelTraj action server finished.");
    }
};
