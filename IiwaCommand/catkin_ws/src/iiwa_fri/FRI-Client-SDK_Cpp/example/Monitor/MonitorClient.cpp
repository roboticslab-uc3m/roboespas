#include <cstdio>
#include <iostream>
#include <iomanip>
#include <string>
#include <cstring> // strstr
#include "MonitorClient.h"

using namespace KUKA::FRI;
//******************************************************************************
MonitorClient::MonitorClient()
{
   printf("MonitorClient initialized:\n");
	first_time=true;
	first_timestampSec=0;
	first_timestampNanosec=0;
}

//******************************************************************************
MonitorClient::~MonitorClient()
{
}
 
void MonitorClient::printArray7(const double* array, std::string name, int totalwidth)
{
	std::cout << std::fixed << std::setprecision(9);
	std::cout << name.c_str() << "1:" << std::setw(totalwidth-strlen(name.c_str())-2) << array[0] << std::endl;
	std::cout << name.c_str() << "2:" << std::setw(totalwidth-strlen(name.c_str())-2) << array[1] << std::endl;
	std::cout << name.c_str() << "3:" << std::setw(totalwidth-strlen(name.c_str())-2) << array[2] << std::endl;
	std::cout << name.c_str() << "4:" << std::setw(totalwidth-strlen(name.c_str())-2) << array[3] << std::endl;
	std::cout << name.c_str() << "5:" << std::setw(totalwidth-strlen(name.c_str())-2) << array[4] << std::endl;
	std::cout << name.c_str() << "6:" << std::setw(totalwidth-strlen(name.c_str())-2) << array[5] << std::endl;
	std::cout << name.c_str() << "7:" << std::setw(totalwidth-strlen(name.c_str())-2) << array[6] << std::endl;
}
     
//******************************************************************************
void MonitorClient::onStateChange(ESessionState oldState, ESessionState newState)
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
void MonitorClient::monitor()
{
   LBRClient::monitor(); 
	LBRState currentRobotState=robotState();
	double sampleTime = currentRobotState.getSampleTime();
	std::string sessionState = SESSION_STATES[currentRobotState.getSessionState()];
	std::string connectionQuality = CONNECTION_QUALITIES[currentRobotState.getConnectionQuality()];
	std::string safetyState = SAFETY_STATES[currentRobotState.getSafetyState()];
	std::string operationMode = OPERATION_MODES[currentRobotState.getOperationMode()];
	std::string driveState = DRIVE_STATES[currentRobotState.getDriveState()];
	std::string clientCommandMode = CLIENT_COMMAND_MODES[currentRobotState.getClientCommandMode()];
	std::string overlayType = OVERLAY_TYPES[currentRobotState.getOverlayType()];
	std::string controlMode = CONTROL_MODES[currentRobotState.getControlMode()];
	int timestampSec = currentRobotState.getTimestampSec();
	int timestampNanosec = currentRobotState.getTimestampNanoSec();
	double trackingPerformance = currentRobotState.getTrackingPerformance();

	//Calculate timestamp, taking first timestamp as 0
	if (first_time)
	{
		first_timestampSec=timestampSec;
		first_timestampNanosec=timestampNanosec;
		first_time=false;
	}
	int timestampSecFromStart=timestampSec-first_timestampSec;
	int timestampNanosecFromStart=timestampNanosec-first_timestampNanosec;
	double timestampFromStart=timestampSecFromStart+1e-9*timestampNanosecFromStart;
		
	const double* measuredJointPosition = currentRobotState.getMeasuredJointPosition();
	const double* commandedJointPosition = currentRobotState.getCommandedJointPosition();
	//const double* ipoJointPosition = currentRobotState.getIpoJointPosition(); //Interpolator commanded joint positions, not available in monitoring mode
	const double* measuredTorque = currentRobotState.getMeasuredTorque();
	const double* commandedTorque = currentRobotState.getCommandedTorque();
	const double* externalTorque = currentRobotState. getExternalTorque();
		
	int totalwidth=50;
	std::cout << "-" << std::setfill(' ') << std::endl;
	std::cout << "SampleTime:" << std::setw(totalwidth-12) << sampleTime << std::endl;
	std::cout << "SessionState:" << std::setw(totalwidth-14) << sessionState << std::endl;
	std::cout << "ConnectionQuality:" << std::setw(totalwidth-19) << connectionQuality << std::endl;
	std::cout << "SafetyState:" << std::setw(totalwidth-13) << safetyState << std::endl;
	std::cout << "OperationMode:" << std::setw(totalwidth-15) << operationMode << std::endl;
	std::cout << "DriveState:" << std::setw(totalwidth-12) << driveState << std::endl;
	std::cout << "ClientCommandMode:" << std::setw(totalwidth-19) << clientCommandMode << std::endl;
	std::cout << "OverlayType:" << std::setw(totalwidth-13) << overlayType << std::endl;
	std::cout << "ControlMode:" << std::setw(totalwidth-13) << controlMode << std::endl;
	std::cout << "TimestampFromStart:" << std::setw(totalwidth-20) << std::fixed << std::setprecision(3) << timestampFromStart << std::endl;
	std::cout << "TrackingPerformance:" << std::setw(totalwidth-21) << trackingPerformance << std::endl;
	std::cout << "Measured joint positions:" << std::endl;		
	this->printArray7(measuredJointPosition, "JP", totalwidth-30);
	std::cout << "Commanded joint positions:" << std::endl;
	this->printArray7(commandedJointPosition, "JP", totalwidth-30);
	//std::cout << "Commanded by interpolator joint positions:" << std::endl; //Not available in monitoring mode
	//printArray7(ipoJointPosition, "JP", totalwidth); //Not available in monitoring mode
	std::cout << "Measured joint torques:" << std::endl;
	this->printArray7(measuredTorque, "JT", totalwidth-30);
	std::cout << "Commanded joint torques:" << std::endl;
	this->printArray7(commandedTorque, "JT", totalwidth-30);
	std::cout << "External joint torques:" << std::endl;
	this->printArray7(externalTorque, "JT", totalwidth-30);   
}

//******************************************************************************
void MonitorClient::waitForCommand()
{
   // In waitForCommand(), the joint values have to be mirrored. Which is done, 
   // by calling the base method.
   LBRClient::waitForCommand();
   
   /***************************************************************************/
   /*                                                                         */
   /*   Place user Client Code here                                           */
   /*                                                                         */
   /***************************************************************************/   
   
}

//******************************************************************************
void MonitorClient::command()
{
   /***************************************************************************/
   /*                                                                         */
   /*   Place user Client Code here                                           */
   /*                                                                         */
   /***************************************************************************/   
   
   // In command(), the joint angle values have to be set. 
   //robotCommand().setJointPosition( newJointValues );
}

/**

The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Deutschland GmbH, Augsburg, Germany.

SCOPE

The software “KUKA Sunrise.Connectivity FRI Client SDK” is targeted to work in
conjunction with the “KUKA Sunrise.Connectivity FastRobotInterface” toolkit.
In the following, the term “software” refers to all material directly
belonging to the provided SDK “Software development kit”, particularly source
code, libraries, binaries, manuals and technical documentation.

COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2018 
KUKA Deutschland GmbH
Augsburg, Germany

LICENSE 

Redistribution and use of the software in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:
a) The software is used in conjunction with KUKA products only. 
b) Redistributions of source code must retain the above copyright notice, this
list of conditions and the disclaimer.
c) Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the disclaimer in the documentation and/or other
materials provided with the distribution. Altered source code of the
redistribution must be made available upon request with the distribution.
d) Modification and contributions to the original software provided by KUKA
must be clearly marked and the authorship must be stated.
e) Neither the name of KUKA nor the trademarks owned by KUKA may be used to
endorse or promote products derived from this software without specific prior
written permission.


DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," without warranty of

any kind, including without limitation the warranties of merchantability,

fitness for a particular purpose and non-infringement. 
KUKA makes no warranty that the Software is free of defects or is suitable for
any particular purpose. In no event shall KUKA be responsible for loss or
damages arising from the installation or use of the Software, including but

not limited to any indirect, punitive, special, incidental or consequential

damages of any character including, without limitation, damages for loss of
goodwill, work stoppage, computer failure or malfunction, or any and all other
commercial damages or losses. 
The entire risk to the quality and performance of the Software is not borne by
KUKA. Should the Software prove defective, KUKA is not liable for the entire
cost of any service and repair.



\file
\version {1.16}
*/
