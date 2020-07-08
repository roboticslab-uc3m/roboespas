#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <string>
#include "ros/ros.h"
#include "ROSFRIClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#define DEFAULT_PORTID 30200

using namespace std;
using namespace KUKA::FRI;

int main(int argc, char **argv)
{
	ros::init(argc,argv, "ROSFRI");
	ros::NodeHandle nh;
	ROS_INFO("Node registered as /ROSFRI");

	// parse command line arguments
	if (argc > 1)
	{
		if ( strstr (argv[1],"help") != NULL)
		{
			printf(
				"\nKUKA Fieldbus access test application\n\n"
				"\tCommand line arguments:\n"
				"\t1) controller hostname (optional)\n"
				"\t2) port ID (optional)\n"
			);
			return 1;
		}
	}
	char* hostname = (argc >= 2) ? argv[1] : NULL;
	int port = (argc >= 3) ? atoi(argv[2]) : DEFAULT_PORTID;

	// create new client
	ROSFRIClient lbrClient(&nh);
	printf("\nEnter Monitor Client Application");

	// create new udp connection
	UdpConnection connection;


	// pass connection and client to a new FRI client application
	ClientApplication app(connection, lbrClient);

	// connect client application to KUKA Sunrise controller
	app.connect(port, hostname);

	// repeatedly call the step routine to receive and process FRI packets
	bool success = true;
	while (success && ros::ok())
	{
		success = app.step();
		// check if we are in IDLE because the FRI session was closed
		if (lbrClient.robotState().getSessionState() == IDLE)
		{
			// We simply quit. Waiting for a FRI new session would be another 
			// possibility.
			break;
		}
		ros::spinOnce();
	}

	// disconnect from controller
	app.disconnect();
	return 0;
}

