#ifndef _KUKA_FRI_MY_LBR_CLIENT_H
#define _KUKA_FRI_MY_LBR_CLIENT_H

#include <boost/bind.hpp>
#include "friLBRClient.h"

#include "ros/ros.h"
#include "iiwa_fri/LBRStateMsg.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/JointState.h"



/**
 * \brief IiwaFRI Client.
 */
class IiwaFRIClient : public KUKA::FRI::LBRClient
{
public:
	/**
	* \brief IiwaFRI Constructor.
	*/
	IiwaFRIClient(ros::NodeHandle *nh);

	/**
	* \brief IiwaFRI Destructor.
	*/
	~IiwaFRIClient();

	/**
	* \brief Callback for FRI state changes.
	*
	* @param oldState
	* @param newStateW
	*/
	virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);

	/**
	* \brief Callback for the IiwaFRI session states 'Monitoring Wait' and 'Monitoring Ready'.
	*/
	virtual void monitor();

	/**
	* \brief Callback for the IiwaFRI session state 'Commanding Wait'.
	*/
	virtual void waitForCommand();

	/**
	* \brief Callback for the IiwaFRI session state 'Commanding Active'.
	*/
	virtual void command();
private:
	// Variables to calculate timestamp
	bool first_time;
	int first_timestampSec;
	int first_timestampNanosec;

	// ROS Variables
	ros::Publisher LBRState_pub;
	ros::Publisher joint_state_pub;
	ros::Subscriber joint_command_sub;
	// PublishLBRState Function
	void publishState();
	void JointCommandCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg);
};

#endif // _KUKA_FRI_MY_LBR_CLIENT_H
