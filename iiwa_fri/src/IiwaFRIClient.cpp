#include <cstdio>
#include <iostream>
#include <iomanip>
#include <string>
#include <cstring> // strstr
#include "IiwaFRIClient.h"

using namespace std;
using namespace KUKA::FRI;
//******************************************************************************
IiwaFRIClient::IiwaFRIClient(ros::NodeHandle *nh)
{
	LBRState_pub = nh->advertise<iiwa_fri::LBRStateMsg>("/iiwa_fri/LBRState", 1);
	joint_state_pub = nh->advertise<sensor_msgs::JointState>("/iiwa_fri/joint_state", 1);
	joint_command_sub = nh->subscribe("/iiwa_fri/joint_command", 1, &IiwaFRIClient::JointCommandCallback, this);

	ROS_INFO("IiwaFRIClient initialized\n");
	first_time=true;
	first_timestampSec=0;
	first_timestampNanosec=0;
}

//******************************************************************************
IiwaFRIClient::~IiwaFRIClient()
{
}

//******************************************************************************
void IiwaFRIClient::onStateChange(ESessionState oldState, ESessionState newState)
{
   LBRClient::onStateChange(oldState, newState);
   // react on state change events
   switch (newState)
   {
      case MONITORING_WAIT:
      {
		  ROS_INFO("Switched to MONITORING_WAIT\n");
          break;
      }
      case MONITORING_READY:
      {
		  ROS_INFO("Switched to MONITORING_READY\n");
          break;
      }
      case COMMANDING_WAIT:
      {
		  ROS_INFO("Switched to COMMANDING_WAIT\n");
		  break;
      }
      case COMMANDING_ACTIVE:
      {
		  ROS_INFO("Switched to COMMANDING_ACTIVE\n");
          break;
      }
      default:
      {
         break;
      }
   }
}

//******************************************************************************
void IiwaFRIClient::monitor()
{

   	LBRClient::monitor();
   	this->publishState();

}

//******************************************************************************
void IiwaFRIClient::waitForCommand()
{
   // In waitForCommand(), the joint values have to be mirrored. Which is done,
   // by calling the base method.
   LBRClient::waitForCommand();
   ROS_INFO("Waiting for Command ");
   if (robotState().getClientCommandMode() == TORQUE)
   {
      //robotCommand().setTorque(_torques);
      ROS_INFO("Set torque");
   }
   this->publishState();
}

//******************************************************************************
void IiwaFRIClient::command()
{
    double commJointPos[LBRState::NUMBER_OF_JOINTS];
	double readJointPos[LBRState::NUMBER_OF_JOINTS];
    memcpy(commJointPos, robotState().getCommandedJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    memcpy(readJointPos, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
	//readJointPos[0]=readJointPos[0]+0.001;
	/*cout << "commJointPos: ";
	for (unsigned int i=0; i<LBRState::NUMBER_OF_JOINTS; i++)
	{
		cout << commJointPos[i] << ", ";
	}
	cout << endl;
	cout << "readJointPos: ";
	for (unsigned int i=0; i<LBRState::NUMBER_OF_JOINTS; i++)
	{
		cout << readJointPos[i] << ", ";
	}
	cout << endl;*/

	robotCommand().setJointPosition(readJointPos);
   	// In command(), the joint angle values have to be set.
   	this->publishState();
}

void IiwaFRIClient::JointCommandCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg)
{
	cout << "joint_command: ";
	for (int j=0; j < msg->positions.size(); j++)
	{
		cout << msg->positions[j] << ", ";
	}
	cout << endl;
}
void IiwaFRIClient::publishState()
{
	//LBR STATE
	iiwa_fri::LBRStateMsg LBRState_msg;
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

	//JOINT STATE
	sensor_msgs::JointState js_msg;
	for (int i = 0; i < 7; i++)
	{
		js_msg.position.push_back(measured_joint_position[i]);
		js_msg.effort.push_back(measured_joint_torque[i]);
	}
	js_msg.name={"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
	js_msg.header.stamp=ros::Time(timestampSecFromStart + 1e-9*timestampNanosecFromStart);
	joint_state_pub.publish(js_msg);
}
