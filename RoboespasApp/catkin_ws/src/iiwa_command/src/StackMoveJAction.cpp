#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <iiwa_command/MoveJAction.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen3/Eigen/Dense>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointTorque.h>

using namespace std;

class StackMoveJAction
{
    protected:
    //Iiwa command action server variables
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<iiwa_command::MoveJAction> as;
    iiwa_command::MoveJFeedback as_feedback;
    iiwa_command::MoveJResult as_result;
    Eigen::VectorXd q_max;
    ros::Publisher q_pub;
    ros::Subscriber q_sub;
    ros::Subscriber qdot_sub;
    ros::Subscriber qtorque_sub;
    sensor_msgs::JointState joint_state;
    double control_step_size=0.5;
    public:

    StackMoveJAction(std::string name) :
    as(nh, name, boost::bind(&StackMoveJAction::callback_MoveJ, this, _1), false) //Create the action server
    {
        as.start();
        std::vector<double> q_max_vec;
        if (!nh.getParam("/iiwa/limits/joint_position", q_max_vec))
        {
            ROS_ERROR("Failed to read '/iiwa/limits/joint_position' on param server");
        }
        q_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_max_vec.data(), q_max_vec.size());
        q_max = q_max*M_PI/180.0;
        if (!nh.getParam("/iiwa_command/control_step_size", control_step_size))
        {
            ROS_ERROR("Failed to read '/iiwa_command/control_step_size' on param server");
        }
        q_pub = nh.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 0);
        q_sub = nh.subscribe("/iiwa/state/JointPosition", 1000, &StackMoveJAction::callback_qSub, this);
        qdot_sub = nh.subscribe("/iiwa/state/JointVelocity", 1000, &StackMoveJAction::callback_qdotSub, this);
        qtorque_sub = nh.subscribe("/iiwa/state/JointTorque", 1000, &StackMoveJAction::callback_qtorqueSub, this);
        ROS_INFO("Action server %s offered.", name.c_str());
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
    void callback_MoveJ(const iiwa_command::MoveJGoalConstPtr &goal)
    {
        ROS_INFO("MoveJ action server started...");
        //Read control_step_size
        if (!nh.getParam("/iiwa_command/control_step_size", control_step_size))
        {
            ROS_ERROR("Failed to read '/iiwa_command/control_step_size' on param server");
        }
        //Variables returned
        trajectory_msgs::JointTrajectory trajectory_commanded;
        std::vector<sensor_msgs::JointState> trajectory_read;
        //Save in an std::vector
        std::vector<double> q_goal_vec=goal->joint_position;
        //Check position is not empty
        if (q_goal_vec.empty())
        {
            ROS_ERROR("Empty joint position");
            as.setSucceeded(as_result);
            return;
        }
        //Transform into Eigen Vector
        Eigen::VectorXd q_goal = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_goal_vec.data(), q_goal_vec.size());
        //Check position is inside the workspace
        Eigen::VectorXd dist_to_outside = q_max - q_goal.cwiseAbs();
        if (dist_to_outside.minCoeff()<=0)
        {
            ROS_ERROR("Goal joint position outside workspace");
            as_result.trajectory_read=trajectory_read;
            as_result.trajectory_commanded=trajectory_commanded;
            as.setSucceeded(as_result);
            return;
        }
        //Send position
        iiwa_msgs::JointPosition jPos;
        jPos.position.a1=q_goal[0];
        jPos.position.a2=q_goal[1];
        jPos.position.a3=q_goal[2];
        jPos.position.a4=q_goal[3];
        jPos.position.a5=q_goal[4];
        jPos.position.a6=q_goal[5];
        jPos.position.a7=q_goal[6];
        q_pub.publish(jPos);
        ros::Time timeStart = ros::Time::now();
        ros::Duration time_from_start = ros::Time::now()-timeStart;
        double timeout = 300.0;
        bool bContinue=true;
        while (time_from_start.toSec() <= timeout && bContinue)
        {
            trajectory_read.push_back(joint_state);
            Eigen::VectorXd q_curr = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(joint_state.position.data(), joint_state.position.size());
            Eigen::VectorXd dist = q_goal - q_curr;
            //Update feedback
            trajectory_msgs::JointTrajectoryPoint point_command;
            for (int i=0; i<q_goal.size(); i++)
            {
                point_command.positions.push_back(q_goal[i]);
            }
            point_command.time_from_start = time_from_start;
            as_feedback.point_commanded = point_command;
            as_feedback.joint_state = joint_state;
            as_feedback.time_from_start = time_from_start.toSec();
            trajectory_commanded.points.push_back(point_command);
            ros::Duration(control_step_size).sleep();
            time_from_start = ros::Time::now()-timeStart;
            as.publishFeedback(as_feedback);
            if (dist.maxCoeff() <= 1e-4)
            {
                bContinue = false;
            }
        }
        as_result.trajectory_commanded = trajectory_commanded;
        as_result.trajectory_read = trajectory_read;
        as.setSucceeded(as_result);
        ROS_INFO("MoveJ action server finished.");
    }
};
