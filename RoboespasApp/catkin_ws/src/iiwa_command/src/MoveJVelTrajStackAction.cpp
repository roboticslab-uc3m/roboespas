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
#include <iiwa_msgs/TimeToDestination.h>

using namespace std;

class MoveJVelTrajStackAction
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
    public:

    MoveJVelTrajStackAction(std::string name) :
    as(nh, name, boost::bind(&MoveJVelTrajStackAction::callback_MoveJVelTraj, this, _1), false) //Create the action server
    {
        as.start();
        ROS_INFO("Action server %s started", name.c_str());
        std::vector<double> q_max_vec;
        if (!nh.getParam("/iiwa/limits/joint_position", q_max_vec))
        {
            ROS_ERROR("Failed to read '/iiwa/limits/joint_position' on param server");
        }
        q_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_max_vec.data(), q_max_vec.size());
        q_max = q_max*M_PI/180.0;
        qdot_pub = nh.advertise<iiwa_msgs::JointVelocity>("/iiwa/command/JointVelocity", 0);
        q_sub = nh.subscribe("/iiwa/state/JointPosition", 1000, &MoveJVelTrajStackAction::callback_qSub, this);
        qdot_sub = nh.subscribe("/iiwa/state/JointVelocity", 1000, &MoveJVelTrajStackAction::callback_qdotSub, this);
        qtorque_sub = nh.subscribe("/iiwa/state/JointTorque", 1000, &MoveJVelTrajStackAction::callback_qtorqueSub, this);

        timeToDestClient=nh.serviceClient<iiwa_msgs::TimeToDestination>("/iiwa/state/timeToDestination");
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
        double v=0.0001;
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
        ROS_INFO("MoveJVelTraj action server active");
        //Variables returned
        trajectory_msgs::JointTrajectory trajectory_commanded;
        std::vector<sensor_msgs::JointState> trajectory_read;
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
        vector<double> direction(7,0.0);
        for (int i=0; i< npoints; i++)
        {
            //Convert vector into iiwa_msgs::JointPosition
            iiwa_msgs::JointVelocity qdot;
            for (int j=0; j<7; j++)
            {
                if (direction[j]==0.0 && trajectory_desired.points[i].velocities[j] > 1e-5)
                {
                    direction[j] = 1;
                }
                else if (direction[j]==0.0 && trajectory_desired.points[i].velocities[j] < -1e-5)
                {
                    direction[j] = -1;
                }
            }
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
        //Publish first message to start the robot
        WakeUpRobot(direction);
        //Start capturing the trajectory
        ros::Time endTime;
        ros::Time startTime=ros::Time::now();
        //Send the messages
        for (int i=0; i < npoints-1; i++)
        {
            trajectory_read.push_back(joint_state);
            qdot_pub.publish(qdot_traj[i]);
            ros::Time endTime = startTime+ros::Duration(stamps[i]);
            ros::Time::sleepUntil(endTime);
        }
        cout << "np: "<< npoints << endl;
        //Stop the robot just in case the trajectory was not well constructed, but it should have a zero at the end to match the number of velocities, positions and stamps.
        iiwa_msgs::JointVelocity qdot_end;
        qdot_end.velocity.a1 = 0;
        qdot_end.velocity.a2 = 0;
        qdot_end.velocity.a3 = 0;
        qdot_end.velocity.a4 = 0;
        qdot_end.velocity.a5 = 0;
        qdot_end.velocity.a6 = 0;
        qdot_end.velocity.a7 = 0;
        qdot_pub.publish(qdot_end);
        ros::Duration(control_step_size).sleep();
        //Keep capturing for 0.5 seconds
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
        ROS_INFO("MoveJVelTraj action server result sent");
    }
};
