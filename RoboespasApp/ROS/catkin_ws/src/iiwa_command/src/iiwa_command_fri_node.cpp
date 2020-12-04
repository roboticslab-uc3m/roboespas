#include <cstdlib>
#include <iostream>
#include <boost/filesystem.hpp>
#include <string>
#include "ros/ros.h"
#include "FRIMoveJTrajAction.cpp"
#include "FRIMoveJTrajCartVelAction.cpp"
#include "FRIStatePublisher.cpp"
//#include "FRICaptureService.cpp"
#include "FRIClient.cpp"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <math.h>

using namespace std;
#define DEFAULT_PORTID 30200
using namespace KUKA::FRI;

int main(int argc, char **argv)
{
	  ros::init(argc,argv, "iiwa_command_fri");
    ROS_INFO("Node registered as %s\n", ros::this_node::getName().c_str());

    ros::NodeHandle nh;

		//Start FRIClient
		int port = DEFAULT_PORTID;
		char* hostname = NULL;
		FRIClient lbrClient("FriClient");
		UdpConnection connection;
		ClientApplication app(connection, lbrClient);
		app.connect(port, hostname);

		FRIStatePublisher state_pub("StatePub", &lbrClient);
		FRIMoveJTrajAction movejtraj_a("MoveJTraj", &lbrClient);
		FRIMoveJTrajCartVelAction movejtrajcartvel_a("MoveJTrajCartVel", &lbrClient);

		//FRICaptureService capture_srv("Capture", *lbrClient);
	  bool success = true;
    ros::AsyncSpinner spinner(8);
    spinner.start();
		//Publish clock
    while (ros::ok())
    {
				state_pub.PublishState();
				success = app.step();
				if (lbrClient.robotState().getSessionState() == IDLE)
				{
						break;
				}
    }
		ros::waitForShutdown();
		app.disconnect();
	  return 0;
}
