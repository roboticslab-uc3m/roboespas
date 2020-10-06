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
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/TimeToDestination.h>

using namespace std;

class MoveJTrajStackAction
{
    protected:
    //Iiwa command action server variables
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<iiwa_command::MoveJTrajAction> as;
    iiwa_command::MoveJTrajFeedback as_feedback;
    iiwa_command::MoveJTrajResult as_result;
    Eigen::VectorXd q_max;
    ros::Publisher q_pub;
    ros::Publisher qdot_pub;
    ros::ServiceClient timeToDestClient;
    ros::Subscriber q_sub;
    ros::Subscriber qdot_sub;
    ros::Subscriber qtorque_sub;
    sensor_msgs::JointState joint_state;
    public:

    MoveJTrajStackAction(std::string name) :
    as(nh, name, boost::bind(&MoveJTrajStackAction::callback_MoveJTraj, this, _1), false) //Create the action server
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
        q_pub = nh.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 0);
        q_sub = nh.subscribe("/iiwa/state/JointPosition", 1000, &MoveJTrajStackAction::callback_qSub, this);
        qdot_sub = nh.subscribe("/iiwa/state/JointVelocity", 1000, &MoveJTrajStackAction::callback_qdotSub, this);
        qtorque_sub = nh.subscribe("/iiwa/state/JointTorque", 1000, &MoveJTrajStackAction::callback_qtorqueSub, this);

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
    void WakeUpRobot()
    {
        //First prepare the robot to move by sending a really low velocity -> Theres a problem when commanding some velocity to the iiwa:  when it is not previously moving and some velocity is commanded, if it is not really reall low, the robot accelerates too much and goes far away from the desired position. Sending a low velocity works around this problem.
        double v=0.0001;
        vector<double> qdot_ini={v, v, v, v, v, v, v};
        iiwa_msgs::JointVelocity qdot_ini_msg;
        qdot_ini_msg.velocity.a1=v;
        qdot_ini_msg.velocity.a2=v;
        qdot_ini_msg.velocity.a3=v;
        qdot_ini_msg.velocity.a4=v;
        qdot_ini_msg.velocity.a5=v;
        qdot_ini_msg.velocity.a6=v;
        qdot_ini_msg.velocity.a7=v;
        qdot_pub.publish(qdot_ini_msg);
        ros::Duration(0.1).sleep();

    }
    void callback_MoveJTraj(const iiwa_command::MoveJTrajGoalConstPtr &goal)
    {
        ROS_INFO("MoveJTraj action server active");
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
        int id = 0;
        trajectory_msgs::JointTrajectoryPoint point_command;
        ros::Duration time_from_start;
        std::vector<double> q_next;
        //Time variables
        WakeUpRobot();
        ros::Time contTime;
        ros::Duration sleepDur;
        ros::Time tStartTraj = ros::Time::now();
        while (id < trajectory_desired.points.size())
        {
            //Check which position should be commanded now
            id = (ros::Time::now()-tStartTraj).toSec()/control_step_size;
            if (id>=trajectory_desired.points.size())
            {
              break;
            }
            //Prepare variables to command
            q_next = trajectory_desired.points[id].positions;
            time_from_start = trajectory_desired.points[id].time_from_start;
            point_command.positions.clear();
            for (int i=0; i<q_next.size(); i++)
            {
                point_command.positions.push_back(q_next[i]);
            }
            iiwa_msgs::JointPosition jPos;
            jPos.position.a1=q_next[0];
            jPos.position.a2=q_next[1];
            jPos.position.a3=q_next[2];
            jPos.position.a4=q_next[3];
            jPos.position.a5=q_next[4];
            jPos.position.a6=q_next[5];
            jPos.position.a7=q_next[6];
            //Fill read position and commanded position
            trajectory_read.push_back(joint_state);
            point_command.time_from_start = ros::Time::now()-tStartTraj;
            trajectory_commanded.points.push_back(point_command);
            //Command
            q_pub.publish(jPos);
            contTime = tStartTraj + ros::Duration(control_step_size*(id+1));
            sleepDur = contTime-ros::Time::now();
            //cout << "id: " << id << " q_next: " << q_next[5] << " t: " << time_from_start.toSec() << "sleep: " << sleepDur.toSec() << endl;
            sleepDur.sleep();
        }
        //Keep capturing for 0.5 seconds
        int nsamples = 0.5/control_step_size;
        nsamples=std::max(nsamples, 1);
        for (int i=0; i<nsamples; i++)
        {
           trajectory_read.push_back(joint_state);
           ros::Duration(control_step_size).sleep();
        }
        //ros::Duration time_passed = ros::Time::now() - tStartTraj;
        //cout << time_passed.toSec() << endl;
        as_result.trajectory_commanded = trajectory_commanded;
        as_result.trajectory_read = trajectory_read;
        as.setSucceeded(as_result);
        ROS_INFO("MoveJTraj action server result sent");
    }
};
