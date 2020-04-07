#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <iiwa_command/IiwaCommandAction.h>
#include <actionlib/server/simple_action_server.h>
#include <Eigen/Dense>
#include <math.h>

using namespace std;

class IiwaCommandNode
{
    protected:
    ros::NodeHandle nh;
    //Iiwa command action server variables
    actionlib::SimpleActionServer<iiwa_command::IiwaCommandAction> as;
    std::string as_name;
    iiwa_command::IiwaCommandFeedback as_feedback;
    iiwa_command::IiwaCommandResult as_result;
    //Iiwa gazebo state subscriber
    ros::Subscriber iiwa_gazebo_state_sub;
    //Iiwa gazebo command publisher
    ros::Publisher iiwa_gazebo_command_pub;
    sensor_msgs::JointState curr_joint_state;

    public:

    IiwaCommandNode(std::string name) :
    as(nh, name, boost::bind(&IiwaCommandNode::callback_iiwa_command, this, _1), false),
    as_name(name) //Create the action server
    {
	    ROS_INFO("Node registered as %s\n", name.c_str());
        as.start();
        ROS_INFO("Action server %s started", name.c_str());
        iiwa_gazebo_command_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/iiwa_gazebo/joint_command", 1000, false);
        iiwa_gazebo_state_sub = nh.subscribe("/iiwa_gazebo/joint_state", 1000, &IiwaCommandNode::callback_iiwa_gazebo_state, this);
    }
    void callback_iiwa_gazebo_state(const sensor_msgs::JointState& iiwa_gazebo_state_msg)
    {
        curr_joint_state=iiwa_gazebo_state_msg;
    }
    void callback_iiwa_command(const iiwa_command::IiwaCommandGoalConstPtr &goal)
    {
        //PD Gains
        Eigen::VectorXd weights(7);
        Eigen::VectorXd Kp(7);
        Eigen::VectorXd Kd(7);
        weights << 0.3, 0.8, 0.6, 0.6, 0.3, 0.2, 0.1;
        Kp = 100*weights;
        Kd = 2*weights;

        //Variables used inside 
        ros::Time tStartTraj;
        std::vector<trajectory_msgs::JointTrajectoryPoint> points = goal->trajectory_desired.points;

        ros::Rate r(1000);
        for (int i=0; i < points.size(); i++)
        {
            //Get current joint state and current stamp
            sensor_msgs::JointState js=curr_joint_state;
            //Fill tstartTraj if i==0
            if (i==0) 
                tStartTraj=js.header.stamp;
            //Calculate current iteration expected duration depending on points
            ros::Duration it_dur=points[i+1].time_from_start - points[i].time_from_start;
            //Get current index depending on how much time has passed since the beginning
            int h=ceil((js.header.stamp.toSec()-tStartTraj.toSec() + 1e-8)/it_dur.toSec())-1;
            //Get variables used as Eigen vectors or ros::Durations
            // - Current q_curr / qdot_curr
            Eigen::VectorXd q_curr = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(js.position.data(), js.position.size());
            Eigen::VectorXd qdot_curr = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned> (js.velocity.data(), js.velocity.size());
            // - Desired tau_des, q_des, qdot_des
            Eigen::VectorXd tau_des = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(points[h].effort.data(), points[h].effort.size());
            Eigen::VectorXd q_des = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned> (points[h].positions.data(), points[h].positions.size());
            Eigen::VectorXd qdot_des = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned> (points[h].velocities.data(), points[h].velocities.size());
            //Calculate tau to correct the desired tau
            Eigen::VectorXd tau_added = Kp.cwiseProduct(q_des - q_curr) + Kd.cwiseProduct(qdot_des - qdot_curr);
            //Sum both taus
            Eigen::VectorXd tau_total = tau_des + tau_added;
            std::vector<double> tau_total_vec(tau_total.data(), tau_total.data() + tau_total.size());
            //Cout
            ROS_INFO("Point %d\n", i);
            ROS_INFO("h: %d\n", h);
            ROS_INFO("Tdiff: %f\n", it_dur.toSec());
            std::cout << "q_curr: " << q_curr.transpose() << std::endl << "qdot_curr: " << qdot_curr.transpose() << std::endl;
            std::cout << "tau_des: " << tau_des.transpose() << std::endl << "q_des: " << q_des.transpose() << std::endl << "qdot_des: " << qdot_des.transpose() << std::endl; 
            std::cout << "tau_added: " << tau_added.transpose() << std::endl;
            std::cout << "tau_total: " << tau_total.transpose() << std::endl;

            //Send tau   
            trajectory_msgs::JointTrajectoryPoint point_published;
            point_published.effort = tau_total_vec;
            iiwa_gazebo_command_pub.publish(point_published);

            //Publish feedback                     
            //as_feedback.joint_state = curr_joint_state;
            //as.publishFeedback(as_feedback);
            ros::Time tStartIt = js.header.stamp;
            ros::Time tEndIt=curr_joint_state.header.stamp;
            ros::Duration tProcess = tEndIt-tStartIt;
            ros::Duration tSleep = it_dur-tProcess;
            std::cout << "time process: " << tProcess.toSec() << std::endl;
            std::cout << "time sleep: " << tSleep.toSec() << std::endl;
            tSleep.sleep();
            //ros::Time time_end = ros::Time::now();
            //r.sleep();
        }
        
        trajectory_msgs::JointTrajectory trajectory_commanded;
        as_result.trajectory_commanded=trajectory_commanded;
        as.setSucceeded(as_result);

        /*// push_back the seeds for the fibonacci sequence
        feedback_.sequence.clear();
        feedback_.sequence.push_back(0);
        feedback_.sequence.push_back(1);

        // publish info to the console for the user
        ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

        // start executing the action
        for(int i=1; i<=goal->order; i++)
        {
            // check that preempt has not been requested by the client
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            }
            feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
            // publish the feedback
            as_.publishFeedback(feedback_);
            // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep();

        }

        if(success)
        {
            result_.sequence = feedback_.sequence;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
        */
    }
};


int main(int argc, char **argv)
{
	ros::init(argc,argv, "iiwa_command");	
    IiwaCommandNode iiwacommand(ros::this_node::getName());

	bool success = true;
	while (ros::ok())
	{
		ros::spinOnce();
	}
	return 0;
}
