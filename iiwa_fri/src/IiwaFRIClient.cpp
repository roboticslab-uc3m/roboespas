#include <cstdio>
#include <iostream>
#include <iomanip>
#include <string>
#include <cstring> // strstr
#include "ROSFRIClient.h"
#include "fricomm/LBRStateMsg.h"

using namespace KUKA::FRI;
//******************************************************************************
ROSFRIClient::ROSFRIClient(ros::NodeHandle *nh)
{
	LBRState_pub = nh->advertise<fricomm::LBRStateMsg>("/ROSFRI/LBRState", 10);
	printf("ROSFRIClient initialized:\n");
	first_time=true;
	first_timestampSec=0;
	first_timestampNanosec=0;
}

//******************************************************************************
ROSFRIClient::~ROSFRIClient()
{
}
     
//******************************************************************************
void ROSFRIClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   // react on state change events
   switch (newState)
   {
      case MONITORING_WAIT:
      {
         break;
      }       
      case MONITORING_READY:
      {
         break;
      }
      case COMMANDING_WAIT:
      {
         break;
      }   
      case COMMANDING_ACTIVE:
      {
         break;
      }   
      default:
      {
         break;
      }
   }
}

//******************************************************************************
void ROSFRIClient::monitor()
{

   	LBRClient::monitor();
   	this->publishLBRState();

}

//******************************************************************************
void ROSFRIClient::waitForCommand()
{
   // In waitForCommand(), the joint values have to be mirrored. Which is done, 
   // by calling the base method.
   LBRClient::waitForCommand();
   ROS_INFO("Command ");
   if (robotState().getClientCommandMode() == TORQUE)
   {
      //robotCommand().setTorque(_torques);
      ROS_INFO("Set torque");
   }
   this->publishLBRState();
   
}

//******************************************************************************
void ROSFRIClient::command()
{
   
   // In command(), the joint angle values have to be set. 
   //robotCommand().setJointPosition( newJointValues );
   ROS_INFO("Command ");
   this->publishLBRState();
}

void ROSFRIClient::publishLBRState()
{
	fricomm::LBRStateMsg LBRState_msg;	
	LBRState currentRobotState=robotState();
	LBRState_msg.sample_time = currentRobotState.getSampleTime();
	LBRState_msg.session_state = SESSION_STATES[currentRobotState.getSessionState()];
	LBRState_msg.connection_quality = CONNECTION_QUALITIES[currentRobotState.getConnectionQuality()];
	LBRState_msg.safety_state = SAFETY_STATES[currentRobotState.getSafetyState()];
	LBRState_msg.operation_mode = OPERATION_MODES[currentRobotState.getOperationMode()];
	LBRState_msg.drive_state = DRIVE_STATES[currentRobotState.getDriveState()];
	LBRState_msg.client_command_mode = CLIENT_COMMAND_MODES[currentRobotState.getClientCommandMode()];
	LBRState_msg.overlay_type = OVERLAY_TYPES[currentRobotState.getOverlayType()];
	LBRState_msg.control_mode = CONTROL_MODES[currentRobotState.getControlMode()];
	LBRState_msg.tracking_performance = currentRobotState.getTrackingPerformance();

	//Calculate timestamp
	int timestampSec = currentRobotState.getTimestampSec();
	int timestampNanosec = currentRobotState.getTimestampNanoSec();
	if (first_time)
	{
		first_timestampSec=timestampSec;
		first_timestampNanosec=timestampNanosec;
		first_time=false;
	}
	int timestampSecFromStart=timestampSec-first_timestampSec;
	int timestampNanosecFromStart=timestampNanosec-first_timestampNanosec;
	LBRState_msg.timestamp=timestampSecFromStart + 1e-9*timestampNanosecFromStart;
	
	//Build measured joint position vector
	const double* measured_joint_position_pointer= currentRobotState.getMeasuredJointPosition();
	std::vector<double> measured_joint_position(measured_joint_position_pointer, measured_joint_position_pointer + 7);
	LBRState_msg.measured_joint_position = measured_joint_position;
	
	//Build commanded joint position vector
	const double* commanded_joint_position_pointer = currentRobotState.getCommandedJointPosition();
	std::vector<double> commanded_joint_position(commanded_joint_position_pointer, commanded_joint_position_pointer + 7);
	LBRState_msg.commanded_joint_position = commanded_joint_position;
	
	//Build measured joint torque vector
	const double* measured_joint_torque_pointer = currentRobotState.getMeasuredTorque();
	std::vector<double> measured_joint_torque(measured_joint_torque_pointer, measured_joint_torque_pointer + 7);
	LBRState_msg.measured_joint_torque = measured_joint_torque;
	
	//Build commanded joint torque vector
	const double* commanded_joint_torque_pointer = currentRobotState.getCommandedTorque();
	std::vector<double> commanded_joint_torque(commanded_joint_torque_pointer, commanded_joint_torque_pointer + 7);
	LBRState_msg.commanded_joint_torque = commanded_joint_torque;
	
	//Build external joint torque vector
	const double* external_joint_torque_pointer = currentRobotState. getExternalTorque();
	std::vector<double> external_joint_torque(external_joint_torque_pointer, external_joint_torque_pointer + 7);
	LBRState_msg.external_joint_torque = external_joint_torque;
		
	//Fill interpolator joint position if not in Monitor Mode
	if (LBRState_msg.session_state!="MONITORING_READY" && LBRState_msg.session_state!="MONITORING_WAIT")
	{
		const double* interpolator_joint_position_pointer = currentRobotState.getIpoJointPosition(); //Interpolator commanded joint positions, not available in monitoring mode
		std::vector<double> interpolator_joint_position(interpolator_joint_position_pointer, interpolator_joint_position_pointer + 7);
		LBRState_msg.interpolator_joint_position = interpolator_joint_position;
	}
	else
	{
		std::vector<double> interpolator_joint_position(7, 0.0);
		LBRState_msg.interpolator_joint_position = interpolator_joint_position;
	}
	
	LBRState_pub.publish(LBRState_msg);
}
