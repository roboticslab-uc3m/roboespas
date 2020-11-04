#ifndef FRI_CLIENT
#define FRI_CLIENT

#include <boost/bind.hpp>
#include "ros/ros.h"
#include "friLBRClient.h"
#include "std_msgs/String.h"


using namespace KUKA::FRI;
using namespace std;

class FRIClient : public KUKA::FRI::LBRClient
{
public:
  //Writable
  double q_command[7]; //Modify to command the robot
  //Readable
  double q_command_read[7]; //See what is currently being commanded, do not modify
  double q_read[7]; //Watch current robot position, do not modify
  double qtorque_read[7]; //Watch current torque, do not modify
  double control_step_size; //Watch control_step_size, do not modify
  ros::Time curr_time_stamp;

private:
  //Times
  bool first_time;
	int first_timestampSec;
	int first_timestampNanosec;

public:
  FRIClient(string name)
  {
      ROS_INFO("FRIClient %s started", name.c_str());
  }
  void updateJointState()
  {
    LBRState currentRobotState=robotState();
    memcpy(q_read, currentRobotState.getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    memcpy(q_command_read, currentRobotState.getCommandedJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    memcpy(qtorque_read, currentRobotState.getMeasuredTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    //Sample time
    control_step_size = currentRobotState.getSampleTime();
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
    curr_time_stamp = ros::Time(timestampSecFromStart + 1e-9*timestampNanosecFromStart);
  }
  virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState)
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
   	  	    memcpy(q_command, q_read, LBRState::NUMBER_OF_JOINTS * sizeof(double));
   		      ROS_INFO("Switched to COMMANDING_WAIT\n");
   		      break;
         }
         case COMMANDING_ACTIVE:
         {
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
  virtual void monitor()
  {
    LBRClient::monitor();
    this->updateJointState();
  }
	virtual void waitForCommand()
  {
    LBRClient::waitForCommand();
    ROS_INFO("Waiting for Command");
  }
	virtual void command()
  {
    this->updateJointState();
    robotCommand().setJointPosition(q_command); //last_q_command);  //Always necessary
  }
};

#endif
