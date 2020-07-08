#ifndef _KUKA_FRI_MY_LBR_CLIENT_H
#define _KUKA_FRI_MY_LBR_CLIENT_H

#include "friLBRClient.h"

#include "ros/ros.h"
#include "fricomm/LBRStateMsg.h"

/**
 * \brief ROSFRI Client.
 */
class ROSFRIClient : public KUKA::FRI::LBRClient
{
public:
	/**
	* \brief ROSFRI Constructor.
	*/
	ROSFRIClient(ros::NodeHandle *nh);

	/** 
	* \brief ROSFRI Destructor.
	*/
	~ROSFRIClient();

	/**
	* \brief Callback for FRI state changes.
	* 
	* @param oldState
	* @param newStateW
	*/
	virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);

	/**
	* \brief Callback for the ROSFRI session states 'Monitoring Wait' and 'Monitoring Ready'.
	*/
	virtual void monitor();

	/**
	* \brief Callback for the ROSFRI session state 'Commanding Wait'.
	*/
	virtual void waitForCommand();

	/**
	* \brief Callback for the ROSFRI session state 'Commanding Active'.
	*/
	virtual void command();
private:
	// Variables to calculate timestamp
	bool first_time;
	int first_timestampSec;
	int first_timestampNanosec;
	
	// ROS Variables
	ros::Publisher LBRState_pub;
	// PublishLBRState Function
	void publishLBRState();
};

#endif // _KUKA_FRI_MY_LBR_CLIENT_H
