#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointPositionVelocity.h>
#include <iiwa_msgs/TimeToDestination.h>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdlib.h>
#include <algorithm>
#include "roboespas_iiwacontrol/SendJointPositionTrajectory.h"
#include "roboespas_iiwacontrol/SendJointVelocityTrajectory.h"
#include "roboespas_iiwacontrol/SendJointPosition.h"
#include "roboespas_iiwacontrol/SendIDKVelocityTrajectory.h"

#include "roboespas_iiwacontrol/StartCapture.h"
#include "roboespas_iiwacontrol/StopCapture.h"
#include "roboespas_iiwacontrol/LoadCapture.h"
#include <geometry_msgs/Twist.h>

#include "capture_lib.cpp"

using namespace std;
//|---------------|
//| MOVEMENT NODE | Node that offers 5 services related with data capture
//|---------------|
// -/send_joint_position -> Moves the robot to a desired joint position
// -/send_joint_position_trajectory -> Moves the robot along a desired joint trajectory, commanding the robot in position and stopping in each point of the trajectory
// -/send_joint_velocity_trajectory -> Moves the robot along a desired trajectory, commanding the robot in velocity, and therefore does not stop between commands
// -/send_IDK_joint_velocity_trajectory -> Moves the robot along a desired cartesian trajectory, it computes first the cartesian velocity needed to follow that trajectory, then computes the needed joint velocities for that cartesian velocities (adding an error correction term), and commands the robot in velocity, and therefore does not stop between commands.

//Global variables:
//Publisher in the topic /iiwa/command/JointPosition
ros::Publisher jPosPub;
//Publisher in the topic /iiwa/command/JointVelocity
ros::Publisher jVelPub;
//Used service that tells the remaining time to reach the commanded position
ros::ServiceClient timeToDestClient;
//Service /roboespas/capture/start, check file "capture.cpp"
ros::ServiceClient startCaptureClient;
//Service /roboespas/capture/stop, check file "capture.cpp"
ros::ServiceClient stopCaptureClient;
//Current position of the iiwa robot, to send the simulation to it
vector<double> q_iiwa(7,0.0);
//Path to load captures
string dataFilePath="";
mutex mutex_q;

bool CaptureStart()
{
    //Function that calls the service /roboespas/capture/start
	roboespas_iiwacontrol::StartCapture startCaptureService;
	startCaptureClient.call(startCaptureService);
	return startCaptureService.response.success;
}
bool CaptureStop()
{
    //Function that calls the service /roboespas/capture/stop
	roboespas_iiwacontrol::StopCapture stopCaptureService;
	stopCaptureClient.call(stopCaptureService);
	return stopCaptureService.response.success;
}
void q_Callback(const iiwa_msgs::JointPosition::ConstPtr& msg)
{
    //Service function that saves the current iiwa position in a global variable that will be after used by the send_joint_velocity_trajectory_IDK
    vector<double> q {msg->position.a1, msg->position.a2, msg->position.a3, msg->position.a4, msg->position.a5, msg->position.a6, msg->position.a7};
    mutex_q.lock();
    q_iiwa=q;
    mutex_q.unlock();
}
vector<double> GetCurrentIiwaPos()
{
    //Function that returns current iiwa pose ensuring the mutex are locked and therefore the q_iiwa pos is not modified while being read
    vector<double> q_curr;
    mutex_q.lock();
    q_curr=q_iiwa;
    mutex_q.unlock();
    return q_curr;
}
bool SendJointPositionService(roboespas_iiwacontrol::SendJointPosition::Request &req, roboespas_iiwacontrol::SendJointPosition::Response &res)
{
    //Service function that receives 7 doubles and builds a message with those numbers and publishes them in /iiwa/command/JointPosition. This service then waits until the service offered by the iiwa "timeToDestination" returns a result near 0, which means the robot has reached the sent position.
	//Create the message to send and fill it
	iiwa_msgs::JointPosition jPos;
	jPos.position.a1=req.a1;
	jPos.position.a2=req.a2;
	jPos.position.a3=req.a3;
	jPos.position.a4=req.a4;
	jPos.position.a5=req.a5;
	jPos.position.a6=req.a6;
	jPos.position.a7=req.a7;
	jPosPub.publish(jPos);
	iiwa_msgs::TimeToDestination timeToDestService;
	//Wait until the remaining time to reach that joint position is under 0.1 seconds and higher than -0.1 to work around the problem that makes the service timeToDestination return a really low number the first few times called. The loop is exited if remainingTime=-999, which means there's an error, or the remainingTime is around 0.
	float remainingTime=1;
	while (remainingTime<-0.1 || remainingTime>0.1)
	{
		//Check the remaining time every iteration
		timeToDestClient.call(timeToDestService);
		remainingTime=timeToDestService.response.remaining_time;
		ros::Duration(0.05).sleep();
		if (remainingTime==-999)
			break;
	}
	//If there was an error, return false
	if (remainingTime==-999)
	{
		res.success=false;
		ROS_INFO("Error");
	}
	else
	{
		res.success=true;
	}
	ROS_INFO("Moved to requested pos");
	return true;
}
bool SendJointPositionTrajectoryService(roboespas_iiwacontrol::SendJointPositionTrajectory::Request &req, roboespas_iiwacontrol::SendJointPositionTrajectory::Response &res)
{
    //Service function that receives the name of some trajectory files and the folder in which it is contained and sends the joint trajectory to the robot through the topic /iiwa/command/JointPosition, capturing the movement done. To save the capture after, call /roboespas/capture/save giving the name and folder desired. IMPORTANT!The robot should already be in the first position of the trajectory when called this service.
    //Load capture
    vector<geometry_msgs::Twist> x_traj;
    vector<geometry_msgs::Twist> xdot_traj;
    vector<double> q_coefs;
    vector<double> qdot_coefs;
    vector<double> qdotdot_coefs;
    vector<double> tKnot;
   	trajectory_msgs::JointTrajectory q_traj=Capture::Load(req.name, req.folder, dataFilePath, x_traj, xdot_traj, tKnot, q_coefs, qdot_coefs, qdotdot_coefs);

    //Build array of iiwa_msgs::JointPosition while building the trajectory_sent variable
    trajectory_msgs::JointTrajectory trajectory_sent=q_traj;
    int npoints=q_traj.points.size();
    vector<iiwa_msgs::JointPosition> q_traj_iiwa;
    vector<ros::Duration> stamps;
    for (int i=0; i<npoints; i++)
    {      
        //Convert vector into iiwa_msgs::JointPosition using Conversions library
        iiwa_msgs::JointPosition q_iiwa = Conversions::StdVectorToIiwaJointPosition(q_traj.points[i].positions);
        q_traj_iiwa.push_back(q_iiwa);
        //Fill stamps variable
        ros::Duration stamp=q_traj.points[i].time_from_start;
        stamps.push_back(stamp);
        trajectory_sent.points[i].time_from_start = stamp;
        //Fill velocities variable although it won't be used to show it is not being commanded
        vector<double> vec7zero(7, 0.0);
        trajectory_sent.points[i].velocities=vec7zero;
    }
    //Start capturing the trajectory
    CaptureStart();
    //Send the messages
    ros::Time startTime=ros::Time::now();
    for (int i=1; i<npoints; i++)
    {
        ros::Time endTime = startTime+ros::Duration(stamps[i]);
        jPosPub.publish(q_traj_iiwa[i]);
        ros::Time::sleepUntil(endTime);
    }
	//Stop the capture
	CaptureStop();
    ROS_INFO("Sent position trajectory to robot");
	//Fill output
	res.success=true;
    res.trajectory_sent=trajectory_sent;
	return true;
}

bool SendIDKVelocityTrajectoryService(roboespas_iiwacontrol::SendIDKVelocityTrajectory::Request &req, roboespas_iiwacontrol::SendIDKVelocityTrajectory::Response &res)
{
    //Service function that receives the name of some trajectory files and the folder in which it is contained and sends the joint velocity trajectory calculated for each point taking into account the desired cartesian velocity and the velocity needed to correct the cartesian position error between real robot posicion and expected robot position (if it was following correctly the trajectory). It commands the robot through the topic /iiwa/command/JointVelocity, while capturing the movement done. To save the capture after, call /roboespas/capture/save giving the name and folder desired. IMPORTANT!The robot should already be in the first position of the trajectory when called this service
    //Load trajectory
    vector<geometry_msgs::Twist> x_traj;
    vector<geometry_msgs::Twist> xdot_traj;
    vector<double> q_coefs;
    vector<double> qdot_coefs;
    vector<double> qdotdot_coefs;
    vector<double> tKnot;
   	trajectory_msgs::JointTrajectory q_traj=Capture::Load(req.name, req.folder, dataFilePath, x_traj, xdot_traj, tKnot, q_coefs, qdot_coefs, qdotdot_coefs);
    //Create some variables
    trajectory_msgs::JointTrajectory trajectory_sent;
    vector<double> q_curr, q_exp, q_commanded, qdot_commanded, qdot_exp;
    trajectory_msgs::JointTrajectoryPoint p;
    trajectory_msgs::JointTrajectory traj_sent;
    ros::Time startTime, endTime;
    ros::Duration stamp, previous_stamp;
    iiwa_msgs::JointVelocity jVel;
    //Output variables
    geometry_msgs::Twist error_xdot, error_xdot_filtered, xdot_commanded, x_commanded;
    vector<geometry_msgs::Twist> errors_xdot, errors_xdot_filtered, xdots_commanded;
    vector<double> q_commanded_vec, x_commanded_vec, qdot_commanded_vec, xdot_commanded_vec, error_xdot_vec, t_vec;
    //Fill those variables
    trajectory_sent.joint_names={"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    q_curr=GetCurrentIiwaPos();
    q_exp=q_traj.points[0].positions;
    qdot_exp={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //First prepare the robot to move by sending a really low velocity -> Theres a problem when commanding some velocity to the iiwa:  when it is not previously moving and some velocity is commanded, if it is not really reall low, the robot accelerates too much and goes far away from the desired position. Sending a low velocity works around this problem
    double v=0.0000001;
    vector<double> qdot_ini={v, v, v, v, v, v, v};
    iiwa_msgs::JointVelocity qdot_ini_msg=Conversions::StdVectorToIiwaJointVelocity(qdot_ini);
    jVelPub.publish(qdot_ini_msg);
    ros::Duration(0.1).sleep();
    ScrewTheory::filter_error.Clear();
    //Start calculating and moving the robot
    for (int i=0; i<x_traj.size()-1; i++)
    {
        //Update stamps
        stamp=q_traj.points[i+1].time_from_start;
        previous_stamp=q_traj.points[i].time_from_start;
        //Update q_curr
        q_curr=GetCurrentIiwaPos();
        //Compute next joint position taking into account:
        // - xdot_traj[i] -> Desired cartesian velocity (theoretical, offline calculation)
        // - Variables to calculate the error:
        //     * x_traj[i] -> Expected cartesian position
        //     * q_curr -> After applying FK will be the real current cartesian position
        //     * (stamp-previous_stamp) -> Time used for computing the offline cartesian velocity, that will be used to calculate the velocity needed to correct the existing error: 
        //          error_xdot=(x_traj[i]-FK(q_curr))/(stamp-previous_stamp)
        //     * k -> The calculated error will be added up to the theoretical cartesian velocity multiplied by a factor k, which will be a small number and is received as a parameter in the service.
        // - Output variables:
        //     * qdot_commanded -> Computed joint velocity to fulfill the input cartesian velocity with the error correction
        //     * error_xdot -> Computed error explained before
        //     * error_xdot_filtered -> Used error for the calculation, that reduces the noise introduced by the current iiwa position variable, which has noise as it is a direct measure from the real robot
        //     * xdot_commanded -> Cartesian velocity that mixes the input desired cartesian velocity with the computed cartesian velocity error.
        q_commanded=ScrewTheory::InverseDifferentialKinematics(q_curr, q_exp, x_traj[i], xdot_traj[i], (stamp-previous_stamp).toSec(), req.k, qdot_commanded, error_xdot, error_xdot_filtered, xdot_commanded, x_commanded);
        //Create the message
        jVel=Conversions::StdVectorToIiwaJointVelocity(qdot_commanded);
        //Start capturing and start time if i==0
        if (i==0)
        {
            //Start capturing once the first velocity has been calculated
            CaptureStart();
            startTime=ros::Time::now();
            endTime = startTime;
            //First iteration do not wait anything before publishing the first message
        }
        //Wait the rest of the time before publishing next message (for first message won't wait, for the rest will wait until the stamp time is reached
        ros::Time::sleepUntil(endTime);
        //Now publish the next message
        jVelPub.publish(jVel);
        //Update the endTime
        endTime=startTime+ros::Duration(stamp);
        // Save commanded variables
        t_vec.push_back(stamp.toSec());
        q_commanded_vec = Conversions::AddStdVectorToStdVector(q_commanded_vec, q_commanded);
        qdot_commanded_vec = Conversions::AddStdVectorToStdVector(qdot_commanded_vec, qdot_commanded);
        x_commanded_vec = Conversions::AddTwistToStdVector(x_commanded_vec, x_commanded);
        xdot_commanded_vec = Conversions::AddTwistToStdVector(xdot_commanded_vec, xdot_commanded);
        error_xdot_vec = Conversions::AddTwistToStdVector(error_xdot_vec, error_xdot);    
        //Update the expected position
        q_exp=q_commanded;
        qdot_exp=qdot_commanded;
    }
    //Stop the capture
    CaptureStop();
    //Stop the robot
    vector<double> final_vel={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    iiwa_msgs::JointVelocity qdot_end=Conversions::StdVectorToIiwaJointVelocity(final_vel);
    jVelPub.publish(qdot_end);
    ros::Duration(0.005).sleep();
    res.t=t_vec;
    res.q_commanded=q_commanded_vec;
    res.x_commanded=x_commanded_vec;
    res.qdot_commanded=qdot_commanded_vec;
    res.xdot_commanded=xdot_commanded_vec;
    res.error_xdot=error_xdot_vec;
    ROS_INFO("Sent cartesian velocity trajectory to robot");
	return true;
}
bool SendJointVelocityTrajectoryService(roboespas_iiwacontrol::SendJointVelocityTrajectory::Request &req, roboespas_iiwacontrol::SendJointVelocityTrajectory::Response &res)
{
    //Service function that receives the name of some trajectory files and the folder in which it is contained and sends the joint velocity trajectory loaded through the topic /iiwa/command/JointVelocity, while capturing the movement done. To save the capture after, call /roboespas/capture/save giving the name and folder desired. IMPORTANT!The robot should already be in the first position of the trajectory when called this service.
    //Load the capture
    vector<geometry_msgs::Twist> x_traj;
    vector<geometry_msgs::Twist> xdot_traj;
    vector<double> q_coefs;
    vector<double> qdot_coefs;
    vector<double> qdotdot_coefs;
    vector<double> tKnot;
   	trajectory_msgs::JointTrajectory q_traj=Capture::Load(req.name, req.folder, dataFilePath, x_traj, xdot_traj, tKnot, q_coefs, qdot_coefs, qdotdot_coefs);
    int npoints=q_traj.points.size();
    vector<iiwa_msgs::JointVelocity> qdot_traj;
    vector<ros::Duration> stamps;
    //First prepare the robot to move by sending a really low velocity -> Theres a problem when commanding some velocity to the iiwa:  when it is not previously moving and some velocity is commanded, if it is not really reall low, the robot accelerates too much and goes far away from the desired position. Sending a low velocity works around this problem.
    double v=0.0000001;
    vector<double> qdot_ini={v, v, v, v, v, v, v};
    iiwa_msgs::JointVelocity qdot_ini_msg=Conversions::StdVectorToIiwaJointVelocity(qdot_ini);
    qdot_traj.push_back(qdot_ini_msg);
    ros::Duration stamp=ros::Duration(0.1);
    stamps.push_back(stamp);
    for (int i=0; i<npoints-1; i++)
    {      
        //Convert vector into iiwa_msgs::JointPosition
        iiwa_msgs::JointVelocity qdot = Conversions::StdVectorToIiwaJointVelocity(q_traj.points[i].velocities);
        qdot_traj.push_back(qdot);
        //Add stamp
        ros::Duration stamp=q_traj.points[i+1].time_from_start; //First stamp is not being used as it is 0.
        stamps.push_back(stamp);
    }
    ros::Time startTime=ros::Time::now();
    ros::Time endTime = startTime+ros::Duration(stamps[0]);
    //Publish first message to start the robot
    jVelPub.publish(qdot_traj[0]);
    ros::Time::sleepUntil(endTime);
    //Start capturing the trajectory
    CaptureStart();
    startTime=ros::Time::now();
    //Send the rest of the messages
    for (int i=1; i<npoints; i++)
    {
        jVelPub.publish(qdot_traj[i]);
        ros::Time endTime = startTime+ros::Duration(stamps[i]);
        ros::Time::sleepUntil(endTime);
    }
    iiwa_msgs::JointVelocity qdot_end = Conversions::StdVectorToIiwaJointVelocity(q_traj.points[npoints-1].velocities); //Should be zero
    jVelPub.publish(qdot_end);
    ros::Duration(0.001).sleep();
	//Stop the capture
	CaptureStop();
    //Stop the robot just in case the trajectory was not well constructed, but it should have a zero at the end to match the number of velocities, positions and stamps.
    vector<double> final_vel(7,0.0);
    qdot_end=Conversions::StdVectorToIiwaJointVelocity(final_vel);
    jVelPub.publish(qdot_end);
    ros::Duration(0.005).sleep();
    //Info
    ROS_INFO("Sent joint velocity trajectory to robot");
	//Fill output
	res.success=true;
    res.trajectory_sent=q_traj;
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "movement");
	ros::NodeHandle n;
	ROS_INFO("Node registered as /movement");
    //LOAD PARAMETERS
    n.getParam("/roboespas/dataPath", dataFilePath);

	//----------------------------------------------------------------------------------------------------------
    //INITIALIZATE SCREW THEORY LIBRARY
	//----------------------------------------------------------------------------------------------------------  
	//Load Twists
    vector<vector<double>> twists;
    for (unsigned int i=1; i<=7; i++)
    {
        vector<double> twist;
        string str="/roboespas/iiwa/twists/t"+to_string(i);
        n.getParam(str, twist);
        twists.push_back(twist);
    }
    //Load Hst0
    vector<double> Hst0_axang;
    vector<double> Hst0_t;
    n.getParam("/roboespas/iiwa/Hst0/axang", Hst0_axang);
    n.getParam("/roboespas/iiwa/Hst0/t", Hst0_t);
    
    //Load joint_pos_limits and joint_vel_limits;
    vector<double> joint_vel_limits(7);
    n.getParam("/roboespas/iiwa/limits/joint_vel", joint_vel_limits);
    vector<double> joint_pos_limits(7);
    n.getParam("/roboespas/iiwa/limits/joint_pos", joint_pos_limits);

    //Load CM, IT, mass
    vector<vector<double>> centersofmass;
    vector<vector<double>> inertias;
    vector<double> masses;
    for (unsigned int i=1; i<=7; i++)
    {
        vector<double> com;
        string strcom="/roboespas/iiwa/links/com/com"+to_string(i);
        n.getParam(strcom, com);
        centersofmass.push_back(com);
        vector<double> inertia;
        string stri="/roboespas/iiwa/links/inertias/i"+to_string(i);
        n.getParam(stri, inertia);
        inertias.push_back(inertia);
        double m;
        string strm="/roboespas/iiwa/links/masses/m"+to_string(i);
        n.getParam(strm, m);
        masses.push_back(m);
    }
	//----------------------------------------------------------------------------------------------------------
    //INITIALIZATE LIBRARY
	//----------------------------------------------------------------------------------------------------------  
    ScrewTheory::Init(Hst0_t, Hst0_axang, twists, joint_pos_limits, joint_vel_limits, centersofmass, inertias, masses);

	//----------------------------------------------------------------------------------------------------------  
	//SUBSCRIBERS
	//----------------------------------------------------------------------------------------------------------
	//iiwa/state/JointPosition
	ros::Subscriber q_Sub = n.subscribe("/iiwa/state/JointPosition", 1, q_Callback);

	//----------------------------------------------------------------------------------------------------------
	//PUBLISHERS
	//----------------------------------------------------------------------------------------------------------
	//iiwa/command/JointPosition
	jPosPub = n.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 0);
	//iiwa/command/JointVelocity
	jVelPub = n.advertise<iiwa_msgs::JointVelocity>("/iiwa/command/JointVelocity", 0);

	//----------------------------------------------------------------------------------------------------------
	//SERVICES USED
	//----------------------------------------------------------------------------------------------------------
	timeToDestClient=n.serviceClient<iiwa_msgs::TimeToDestination>("/iiwa/state/timeToDestination");
	startCaptureClient=n.serviceClient<roboespas_iiwacontrol::StartCapture>("/roboespas/capture/start");
	stopCaptureClient=n.serviceClient<roboespas_iiwacontrol::StopCapture>("/roboespas/capture/stop");

	//----------------------------------------------------------------------------------------------------------
	//SERVICES OFFERED
	//----------------------------------------------------------------------------------------------------------
	//send_joint_position_trajectory
	ros::ServiceServer sendPositionTrajectoryServer=n.advertiseService("/roboespas/movement/send_joint_position_trajectory", SendJointPositionTrajectoryService);
	ROS_INFO("Service offered /roboespas/movement/send_joint_position_trajectory");
	//send_joint_velocity_trajectory
	ros::ServiceServer sendVelocityTrajectoryServer=n.advertiseService("/roboespas/movement/send_joint_velocity_trajectory", SendJointVelocityTrajectoryService);
	ROS_INFO("Service offered /roboespas/movement/send_joint_velocity_trajectory");
	//send_joint_position
	ros::ServiceServer sendPositionServer=n.advertiseService("/roboespas/movement/send_joint_position", SendJointPositionService);
	ROS_INFO("Service offered /roboespas/movement/send_joint_position");
    //send_IDK_joint_velocity_trajectory
	ros::ServiceServer sendIDKVelocityTrajectoryServer=n.advertiseService("/roboespas/movement/send_IDK_joint_velocity_trajectory", SendIDKVelocityTrajectoryService);
	ROS_INFO("Service offered /roboespas/movement/send_IDK_joint_velocity_trajectory");
	//----------------------------------------------------------------------------------------------------------

	//Loop with async spinner to read current position while executing a service
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
	return 0;
}
