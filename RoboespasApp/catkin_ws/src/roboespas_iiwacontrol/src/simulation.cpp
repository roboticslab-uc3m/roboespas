#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <iiwa_msgs/JointPosition.h>
#include <fstream>
#include <iostream>
#include "roboespas_iiwacontrol/SendCurrentIiwaPositionToGazebo.h"
#include "roboespas_iiwacontrol/SendJointPositionGazebo.h"
#include "roboespas_iiwacontrol/SendJointTrajectoryGazebo.h"
#include "capture_lib.cpp"
//|-----------------|
//| SIMULATION NODE | Node that offers 3 services related with gazebo simulator
//|-----------------|
// - /roboespas/simulation/send_joint_position -> Sends a given joint position to the gazebo robot simulator
// - /roboespas/simulation/send_current_iiwa_position -> Reads the current real IIWA position and sends it to the gazebo robot simulator
// - /roboespas/simulation/send_joint_trajectory -> Sends a whole joint trajectory to the gazebo robot simulator

//Global variables:
//Current position of the iiwa robot, to send the simulation to it
iiwa_msgs::JointPosition currIiwaPos;
//Current position of the gazebo simulation, to check whether or not it has reached the commanded position
control_msgs::JointTrajectoryControllerState currGazeboPos;
//Publisher of the position in gazebo
ros::Publisher gazeboTrajPub;
//For loading captures without using serviec
string dataFilePath="";


void iiwaJointPositionCallback(const iiwa_msgs::JointPosition::ConstPtr& msg)
{
    //Callback that reads and saves in a variable current real iiwa position in case the service 
	currIiwaPos=*msg;
	//
	//END of suscriber /iiwa/state/JointPosition
	//----------------------------------------------------------------------------------------------------------
}
void gazeboJointPositionCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
    //Callback that reads and saves in a variable current simulation robot used when trying to know if the robot has reached some position	
    currGazeboPos=*msg;
}
void sendJPos(std::vector<double> positions)
{
    //Function that publishes the desired position in the gazebo command position topic and waits for the simulation to reach the position. Used by two services: /roboespas/simulation/send_joint_position and /roboespas/simulation/send_current_iiwa_position
	//Build a JointTrajectory with just one point whose position is the position given and whose velocities (the joint velocities desired when it reaches that position) are 0s
	std::vector<double> velocities={0,0,0,0,0,0,0};
	trajectory_msgs::JointTrajectory pubTraj;
	pubTraj.joint_names=currGazeboPos.joint_names;
	trajectory_msgs::JointTrajectoryPoint pubPos;
	pubPos.positions=positions;
	pubPos.velocities=velocities;
	pubPos.time_from_start=ros::Duration(1);
	pubTraj.points.push_back(pubPos);
	//Publish the build trajectory in the topic
	gazeboTrajPub.publish(pubTraj);
	//Wait until the errors for each joint are low
	double error=5e-5;
	bool reached=false;
	ros::Duration(0.1).sleep();
	while (!reached)
	{	
		ros::Duration(0.1).sleep();
        //TODO: Why these errors work?
		bool r1=fabs(currGazeboPos.error.positions[0])<=error;
		bool r2=fabs(currGazeboPos.error.positions[1])<=error*10;
		bool r3=fabs(currGazeboPos.error.positions[2])<=error*10;
		bool r4=fabs(currGazeboPos.error.positions[3])<=error*10;
		bool r5=fabs(currGazeboPos.error.positions[4])<=error*5;
		bool r6=fabs(currGazeboPos.error.positions[5])<=error*5;
		bool r7=fabs(currGazeboPos.error.positions[6])<=error*10;
		reached=r1 and r2 and r3 and r4 and r5 and r6 and r7;
	}
	return;
}

bool sendJointTrajectory(roboespas_iiwacontrol::SendJointTrajectoryGazebo::Request &req, roboespas_iiwacontrol::SendJointTrajectoryGazebo::Response &res)
{
    //Service function that receives the name of some trajectory files and the folder in which it is contained, and loads the joint trajectory and time stamps using the function Load inside the capture_lib file and then moves the robot simulator through the points in the trajectory. 
    //Load the capture
    std::vector<geometry_msgs::Twist> x_traj, xdot_traj;
    std::vector<double> q_coefs, qdot_coefs, qdotdot_coefs, tKnot;
   	trajectory_msgs::JointTrajectory q_traj=Capture::Load(req.name, req.folder, dataFilePath, x_traj, xdot_traj, tKnot, q_coefs, qdot_coefs, qdotdot_coefs);
	//For each point, build a new trajectory and fill it with just one point, then send it
	int npoints=q_traj.points.size();
	trajectory_msgs::JointTrajectory pubTraj;
	pubTraj.joint_names=currGazeboPos.joint_names;
	ros::Time startTime=ros::Time::now();
	ros::Duration startStamp=ros::Duration(startTime.sec, startTime.nsec);
	for (int i=0; i<npoints; i+=10)
	{
		trajectory_msgs::JointTrajectoryPoint pubPos;
		pubPos.positions=q_traj.points[i].positions;
		pubPos.time_from_start=ros::Duration(0.2); //TODO: Why does this work? Tried to use ros::Duration(ros::Time::now().sec, ros::Time::now().nsec)+ros::Duration(q_traj.points[i].time_from_start but doesn't work
		pubTraj.points.clear();
		pubTraj.points.push_back(pubPos);
		gazeboTrajPub.publish(pubTraj);
		//Wait the expected time for that point, given inside the loaded trajectory
		ros::Time continueTime=startTime+ros::Duration(q_traj.points[i].time_from_start);
		ros::Time::sleepUntil(continueTime);
	}
	//Calculate elapsed time to compare with the elapsed time when sendint the trajectory to the real IIWA
	ros::Time endTime=ros::Time::now();
	ros::Duration elapsed=endTime-startTime;
	ROS_INFO("Elapsed time: %f", elapsed.toSec());
	ROS_INFO("Sent trajectory to gazebo");
	//Return success
	res.success=true;
	return true;
}

bool sendJointPosition(roboespas_iiwacontrol::SendJointPositionGazebo::Request &req, roboespas_iiwacontrol::SendJointPositionGazebo::Response &res)
{
	//Service function that receives 7 float numbers and sends the gazebo robot to that joint position
	//Build the vector
	std::vector<double> positions={req.a1, req.a2, req.a3, req.a4, req.a5, req.a6, req.a7};
	//Call the previously explained function
	sendJPos(positions);
	res.success=true;
	ROS_INFO("Moved iiwa gazebo to requested pos");
	return true;
}



bool sendCurrentIiwaPosition(roboespas_iiwacontrol::SendCurrentIiwaPositionToGazebo::Request &req, roboespas_iiwacontrol::SendCurrentIiwaPositionToGazebo::Response &res)
{
	//Service function that moves the gazebo robot to the current Iiwa position
	//Build the vector
	std::vector<double> positions={currIiwaPos.position.a1, currIiwaPos.position.a2, currIiwaPos.position.a3, currIiwaPos.position.a4, currIiwaPos.position.a5, currIiwaPos.position.a6, currIiwaPos.position.a7};
	//Call the previously explained function
	sendJPos(positions);
	res.success=true;
	ROS_INFO("Moved robot to current Iiwa position.");
	return true;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "simulation");
	ros::NodeHandle n;
	ROS_INFO("Node registered as /simulation");
    //LOAD PARAMETERS
    n.getParam("/roboespas/dataPath", dataFilePath);

	//PUBLISHERS
	//----------------------------------------------------------------------------------------------------------
	//iiwa/PositionJointInterface_trajectory_controller/command
	gazeboTrajPub = n.advertise<trajectory_msgs::JointTrajectory>("/iiwa/PositionJointInterface_trajectory_controller/command", 1000);
	//----------------------------------------------------------------------------------------------------------
	
	//SUBSCRIBERS
	//----------------------------------------------------------------------------------------------------------
	//iiwa/state/JointPosition
	ros::Subscriber currPosSub = n.subscribe("/iiwa/state/JointPosition",1,iiwaJointPositionCallback);
	//iiwa/PositionJointInterface_trajectory_controller/state
	ros::Subscriber gazeboPosSub = n.subscribe("/iiwa/PositionJointInterface_trajectory_controller/state",1,gazeboJointPositionCallback);
	//----------------------------------------------------------------------------------------------------------

	//SERVICES OFFERED
	//----------------------------------------------------------------------------------------------------------
	//roboespas/simulation/send_joint_trajectory
	ros::ServiceServer sendTrajectoryByNameServer=n.advertiseService("/roboespas/simulation/send_joint_trajectory", sendJointTrajectory);
	ROS_INFO("Service offered /roboespas/simulation/send_joint_trajectory");
	//roboespas/simulation/send_joint_position
	ros::ServiceServer sendPositionServer=n.advertiseService("/roboespas/simulation/send_joint_position", sendJointPosition);
	ROS_INFO("Service offered /roboespas/simulation/send_joint_position");
	//roboespas/simulation/send_current_iiwa_position
	ros::ServiceServer sendIiwaPositionServer=n.advertiseService("/roboespas/simulation/send_current_iiwa_position", sendCurrentIiwaPosition);
	ROS_INFO("Service offered /roboespas/simulation/send_current_iiwa_position");
	//----------------------------------------------------------------------------------------------------------

	//Async spinner so current joint positions (real robot and gazebo robot) keep updating although any service is being executed
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
	return 0;
}
