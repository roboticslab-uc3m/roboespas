#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <actionlib/server/simple_action_server.h>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include "IiwaCommandFriNode.cpp"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <math.h>
#define DEFAULT_PORTID 30200

using namespace std;
using namespace KUKA::FRI;

int main(int argc, char **argv)
{
		//INIT NODE
		ros::init(argc,argv, "iiwa_command_fri");
    ROS_INFO("Node registered as %s\n", ros::this_node::getName().c_str());

		//START FRI Client
		int port = DEFAULT_PORTID;
		char* hostname = NULL;
		IiwaCommandFriNode lbrClient("IiwaCommandFriNode");
		UdpConnection connection;
		ClientApplication app(connection, lbrClient);
		app.connect(port, hostname);

		ros::AsyncSpinner spinner(8);
		spinner.start();
		bool success = true;
		while (ros::ok())
		{
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
