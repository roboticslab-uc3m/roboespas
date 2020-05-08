#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <iiwa_command/IiwaCommandAction.h>
#include <actionlib/server/simple_action_server.h>
#include <Eigen/Dense>
#include <math.h>
#include "IiwaCommandAction.cpp"
#include "MoveJAction.cpp"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc,argv, "iiwa_command");	
    ROS_INFO("Node registered as %s\n", ros::this_node::getName().c_str());
    IiwaCommandAction iiwacommand(ros::this_node::getName());
    MoveJAction movej("MoveJ");

	bool success = true;
	while (ros::ok())
	{
		ros::spinOnce();
	}
	return 0;
}
