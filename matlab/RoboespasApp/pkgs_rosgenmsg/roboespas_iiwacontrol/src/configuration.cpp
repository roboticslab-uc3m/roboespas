#include "ros/ros.h"
#include "roboespas_iiwacontrol/SetImpedanceControlMode.h"
#include "roboespas_iiwacontrol/SetPositionControlMode.h"
#include <iiwa_msgs/ConfigureSmartServo.h>
#include <iiwa_msgs/ControlMode.h>

//|--------------------|
//| CONFIGURATION NODE | Node that offers 2 services related with robot configuration
//|--------------------|
// - /roboespas/configuration/impedance_control_mode -> Impedance control mode with prestablished values of stifness and damping that ensure the user may move the robot freely
// - /roboespas/configuration/position_control_mode -> Position control mode that restores the robot to its normal control mode and allows user to command movements

//Global variables:
//Service to configure control mode /iiwa/configuration/ConfigureControlMode
ros::ServiceClient configServiceClient;

bool setImpedanceControlMode(roboespas_iiwacontrol::SetImpedanceControlMode::Request &recReq, roboespas_iiwacontrol::SetImpedanceControlMode::Response &sentRes)
{
	//Service function that sets the joint impedance control mode, freeing the breaks and letting the user to move the robot smoothly.
	iiwa_msgs::ConfigureSmartServo sentReq;
	iiwa_msgs::ControlMode mode;
	sentReq.request.control_mode=mode.JOINT_IMPEDANCE;
	sentReq.request.joint_impedance.joint_damping.a1=0.7;
	sentReq.request.joint_impedance.joint_damping.a1=0.7;
	sentReq.request.joint_impedance.joint_damping.a1=0.7;
	sentReq.request.joint_impedance.joint_damping.a1=0.7;
	sentReq.request.joint_impedance.joint_damping.a1=0.7;
	sentReq.request.joint_impedance.joint_damping.a1=0.7;
	sentReq.request.joint_impedance.joint_damping.a1=0.7;
	sentReq.request.joint_impedance.joint_stiffness.a1=0.5;
	sentReq.request.joint_impedance.joint_stiffness.a2=0.5;
	sentReq.request.joint_impedance.joint_stiffness.a3=0.5;
	sentReq.request.joint_impedance.joint_stiffness.a4=0.5;
	sentReq.request.joint_impedance.joint_stiffness.a5=0.5;
	sentReq.request.joint_impedance.joint_stiffness.a6=0.5;
	sentReq.request.joint_impedance.joint_stiffness.a7=0.5;
	//Send message and check if the service was successful
	if (configServiceClient.call(sentReq))
	{
		sentRes.success=true;
		ROS_INFO("Set impedance control mode true");
	}
	else
	{
		sentRes.success=false;
		ROS_INFO("Set impedance control mode false");
	}
	return true;
}
bool setPositionControlMode(roboespas_iiwacontrol::SetPositionControlMode::Request &recReq, roboespas_iiwacontrol::SetPositionControlMode::Response &sentRes)
{
    //Service function that sets the position control mode, stopping the joint impedance control and returning to the normal control robot mode.
	iiwa_msgs::ConfigureSmartServo sentReq;
	iiwa_msgs::ControlMode mode;
	sentReq.request.control_mode=mode.POSITION_CONTROL;
	//Send message and check if the service was successful
	if (configServiceClient.call(sentReq))
	{
		sentRes.success=true;
		ROS_INFO("Set position control mode true");
	}
	else
	{
		sentRes.success=false;
		ROS_INFO("Set position control mode false");
	}
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "configuration");
	ros::NodeHandle n;
	ROS_INFO("Node registered as /configuration");

	//SERVICES USED
	//----------------------------------------------------------------------------------------------------------
	//iiwa/configuration/ConfigureControlMode
configServiceClient=n.serviceClient<iiwa_msgs::ConfigureSmartServo>("/iiwa/configuration/ConfigureControlMode");
	//----------------------------------------------------------------------------------------------------------

	//SERVICES OFFERED
	//----------------------------------------------------------------------------------------------------------
	//roboespas/configuration/impedance_control_mode
	ros::ServiceServer impedanceControlServer=n.advertiseService("/roboespas/configuration/impedance_control_mode", setImpedanceControlMode);
	ROS_INFO("Service offered /roboespas/configuration/impedance_control_mode");
	//roboespas/configuration/position_control_mode
	ros::ServiceServer positionControlServer=n.advertiseService("/roboespas/configuration/position_control_mode", setPositionControlMode);
	ROS_INFO("Service offered /roboespas/configuration/position_control_mode");
	//----------------------------------------------------------------------------------------------------------
	
	//Loop
	while (ros::ok())
	{
		ros::spinOnce();
	}
	return 0;
}
