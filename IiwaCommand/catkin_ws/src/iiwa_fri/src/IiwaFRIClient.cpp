#include <cstdio>
#include <iostream>
#include <iomanip>
#include <string>
#include <cstring> // strstr
#include "IiwaFRIClient.h"
#include "std_msgs/String.h"

using namespace std;
using namespace KUKA::FRI;
//******************************************************************************
IiwaFRIClient::IiwaFRIClient(ros::NodeHandle *nh_)
{
	nh = nh_;
	LBRState_pub = nh->advertise<iiwa_fri::LBRStateMsg>("/iiwa_fri/LBRState", 1000);
	joint_state_pub = nh->advertise<sensor_msgs::JointState>("/iiwa_fri/joint_state", 1000);
	info_pub = nh->advertise<std_msgs::String>("/iiwa_fri/info", 1000);
	joint_command_sub = nh->subscribe("/iiwa_fri/joint_command", 1000, &IiwaFRIClient::JointCommandCallback, this);

	ROS_INFO("IiwaFRIClient initialized\n");

	//Initialize variables
	first_time=true;
	first_timestampSec=0;
	first_timestampNanosec=0;
	//Read qdot_max
	std::vector<double> qdot_max_vec;
	if (!nh->getParam("/iiwa/limits/joint_velocity", qdot_max_vec))
	{
		ROS_ERROR("Failed to read '/iiwa/limits/joint_velocity' on param server");
	}
	std::copy(qdot_max_vec.begin(), qdot_max_vec.end(), qdot_max);
	//Read control_step_size from parameter server
	if (!nh->getParam("/iiwa_command/control_step_size", control_step_size))
	{
		ROS_ERROR("Failed to read '/iiwa_command/control_step_size' on param server");
	}
	//Calculate qinc_max with current qdotmax and current control_step_size
	for (int j=0; j<7; j++)
	{
		qinc_max[j] = qdot_max[j]*control_step_size*M_PI/180.0; //Convert to radians and multiply by control_step_size
	}
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
		  memcpy(last_q_command, q_read, LBRState::NUMBER_OF_JOINTS * sizeof(double));
	  	  memcpy(q_command, q_read, LBRState::NUMBER_OF_JOINTS * sizeof(double));
		  ROS_INFO("Switched to COMMANDING_WAIT\n");
		  break;
      }
      case COMMANDING_ACTIVE:
      {
		  //Set q_command to the current once
		  memcpy(q_command, q_read, LBRState::NUMBER_OF_JOINTS * sizeof(double));
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
   this->publishState();
}

//******************************************************************************
void IiwaFRIClient::command()
{

	//Check difference between q_command and q_read
	double diff[7];
	bool unreachable = false;
	bool steady = true;
	//cout << "diff: ";
	for (unsigned int i=0; i<7; i++)
	{
		diff[i] = abs(q_command[i] - last_q_command[i]);

		if ((diff[i]-qinc_max[i])>1e-10)
		{
			unreachable=true;
		}
		if (diff[i]>1e-10)
		{
			cout << diff[i] << ", ";
			steady = false;
		}
	}
	if (steady==false)
	{
		cout << endl;
	}
	//If the robot cannot reach the position or the difference is so little that the robot is considered to be steady
	if (unreachable == true || steady == true)
	{
		std_msgs::String msg;
		std::stringstream ss;
		if (unreachable)
		{
			ss << "unreachable, ";
		}
		else if (steady)
		{
			ss << "steady, ";
		}
    	ss << "qinc_max: " << qinc_max[0] << ", " << qinc_max[1] << ", " << qinc_max[2] << ", " << qinc_max[3] << ", " << qinc_max[4] << ", " << qinc_max[5] << ", " << qinc_max[6];
    	msg.data = ss.str();
		info_pub.publish(msg);
	}
	else
	{
		memcpy(last_q_command, q_command, LBRState::NUMBER_OF_JOINTS*sizeof(double));
		std_msgs::String msg;
		std::stringstream ss;
    	ss << "commanding, qinc_max: " << qinc_max[0] << ", " << qinc_max[1] << ", " << qinc_max[2] << ", " << qinc_max[3] << ", " << qinc_max[4] << ", " << qinc_max[5] << ", " << qinc_max[6] << std::endl;
		ss << "diff: " << diff[0] << ", " << diff[1] << ", " << diff[2] << ", " << diff[3] << ", " << diff[4] << ", " << diff[5] << ", " << diff[6];
    	msg.data = ss.str();
		info_pub.publish(msg);
	}
	robotCommand().setJointPosition(last_q_command);

   	// In command(), the joint angle values have to be set.
   	this->publishState();
}

void IiwaFRIClient::JointCommandCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg)
{
	//Update qinc_max with control_step_size
	//Read control_step_size from parameter server
	if (!nh->getParam("/iiwa_command/control_step_size", control_step_size))
	{
		ROS_ERROR("Failed to read '/iiwa_command/control_step_size' on param server");
	}
	//Calculate qinc_max with current GetVelocity
	for (int j=0; j<7; j++)
	{
		qinc_max[j] = qdot_max[j]*control_step_size*M_PI/180.0; //Convert to radians and multiply by control_step_size
	}
	//Fill q_command;
	for (int j=0; j < msg->positions.size(); j++)
	{
		q_command[j] = msg->positions[j];
	}
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
	memcpy(q_read, currentRobotState.getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
	std::vector<double> measured_joint_position(begin(q_read), end(q_read));
	LBRState_msg.measured_joint_position = measured_joint_position;

	//Build commanded joint position vector
	memcpy(q_command_read, currentRobotState.getCommandedJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
	std::vector<double> commanded_joint_position(begin(q_command_read), end(q_command_read));
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

	//control_step_size
	if (control_step_size!=LBRState_msg.sample_time)
	{
		control_step_size = LBRState_msg.sample_time;
		nh->setParam("/iiwa_command/control_step_size", control_step_size);
		//Calculate qinc_max with current qdotmax and current control_step_size
		for (int j=0; j<7; j++)
		{
			qinc_max[j] = qdot_max[j]*control_step_size*M_PI/180.0; //Convert to radians and multiply by control_step_size
		}
	}

}
