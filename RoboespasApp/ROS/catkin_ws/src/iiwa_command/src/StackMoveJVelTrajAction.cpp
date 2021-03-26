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


using namespace std;

class StackMoveJVelTrajAction
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
    bool capture = false;
    std::vector<sensor_msgs::JointState> trajectory_read;
    public:

    StackMoveJVelTrajAction(std::string name) :
    as(nh, name, boost::bind(&StackMoveJVelTrajAction::callback_MoveJVelTraj, this, _1), false) //Create the action server
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
        q_sub = nh.subscribe("/iiwa/state/JointPosition", 1000, &StackMoveJVelTrajAction::callback_qSub, this);
        qdot_sub = nh.subscribe("/iiwa/state/JointVelocity", 1000, &StackMoveJVelTrajAction::callback_qdotSub, this);
        qtorque_sub = nh.subscribe("/iiwa/state/JointTorque", 1000, &StackMoveJVelTrajAction::callback_qtorqueSub, this);
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
        if (capture)
        {
          trajectory_read.push_back(joint_state);
        }
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
    void callback_MoveJVelTraj(const iiwa_command::MoveJVelTrajGoalConstPtr &goal)
    {
        ROS_INFO("MoveJVelTraj action server started...");
        //Variables returned
        trajectory_msgs::JointTrajectory trajectory_commanded;
        trajectory_msgs::JointTrajectory trajectory_desired = goal->trajectory_desired;
        //Read control_step_size
        double control_step_size;
        if (!nh.getParam("/iiwa_command/control_step_size", control_step_size))
        {
            ROS_ERROR("Failed to read '/iiwa_command/control_step_size' on param server");
        }
        //Check trajectory is not empty
        if (trajectory_desired.points.empty())
        {
            ROS_ERROR("Empty joint trajectory");
            as_result.trajectory_commanded = trajectory_commanded;
            as_result.trajectory_read = trajectory_read;
            as.setSucceeded(as_result);
            return;
        }

        int npoints=trajectory_desired.points.size();
        vector<iiwa_msgs::JointVelocity> qdot_traj;
        vector<ros::Duration> stamps;
        double v=0.0000001;
        vector<double> qdot_ini={v, v, v, v, v, v, v};
        iiwa_msgs::JointVelocity qdot_ini_msg;
        qdot_ini_msg.velocity.a1 = qdot_ini[0];
        qdot_ini_msg.velocity.a2 = qdot_ini[1];
        qdot_ini_msg.velocity.a3 = qdot_ini[2];
        qdot_ini_msg.velocity.a4 = qdot_ini[3];
        qdot_ini_msg.velocity.a5 = qdot_ini[4];
        qdot_ini_msg.velocity.a6 = qdot_ini[5];
        qdot_ini_msg.velocity.a7 = qdot_ini[6];
        qdot_traj.push_back(qdot_ini_msg);
        ros::Duration stamp=ros::Duration(0.1);
        stamps.push_back(stamp);
        for (int i=0; i<npoints-1; i++)
        {
            //Convert vector into iiwa_msgs::JointPosition
            iiwa_msgs::JointVelocity qdot;
            qdot.velocity.a1 = trajectory_desired.points[i].velocities[0];
            qdot.velocity.a2 = trajectory_desired.points[i].velocities[1];
            qdot.velocity.a3 = trajectory_desired.points[i].velocities[2];
            qdot.velocity.a4 = trajectory_desired.points[i].velocities[3];
            qdot.velocity.a5 = trajectory_desired.points[i].velocities[4];
            qdot.velocity.a6 = trajectory_desired.points[i].velocities[5];
            qdot.velocity.a7 = trajectory_desired.points[i].velocities[6];
            qdot_traj.push_back(qdot);
            //Add stamp
            ros::Duration stamp=trajectory_desired.points[i+1].time_from_start; //First stamp is not being used as it is 0.
            stamps.push_back(stamp);
        }
        ros::Time startTime=ros::Time::now();
        ros::Time endTime = startTime+ros::Duration(stamps[0]);
        //Publish first message to start the robot
        qdot_pub.publish(qdot_traj[0]);
        ros::Time::sleepUntil(endTime);
        //Start capturing the trajectory
        trajectory_read.clear();
        capture = true;
        startTime=ros::Time::now();
        //Send the rest of the messages
        for (int i=1; i<npoints; i++)
        {
            qdot_pub.publish(qdot_traj[i]);
            ros::Time endTime = startTime+ros::Duration(stamps[i]);
            ros::Time::sleepUntil(endTime);
        }
        iiwa_msgs::JointVelocity qdot_end;
        qdot_end.velocity.a1 = trajectory_desired.points[npoints-1].velocities[0];
        qdot_end.velocity.a2 = trajectory_desired.points[npoints-1].velocities[1];
        qdot_end.velocity.a3 = trajectory_desired.points[npoints-1].velocities[2];
        qdot_end.velocity.a4 = trajectory_desired.points[npoints-1].velocities[3];
        qdot_end.velocity.a5 = trajectory_desired.points[npoints-1].velocities[4];
        qdot_end.velocity.a6 = trajectory_desired.points[npoints-1].velocities[5];
        qdot_end.velocity.a7 = trajectory_desired.points[npoints-1].velocities[6];
        qdot_pub.publish(qdot_end);
        //Keep capturing for 0.5 seconds
        ros::Duration(0.5).sleep();
        capture = false;
        ros::Duration(1).sleep();
        as_result.trajectory_commanded = trajectory_desired;
        as_result.trajectory_read = trajectory_read;
        ROS_INFO("MoveJVelTraj action server finished.");
        as.setSucceeded(as_result);
    }
};
