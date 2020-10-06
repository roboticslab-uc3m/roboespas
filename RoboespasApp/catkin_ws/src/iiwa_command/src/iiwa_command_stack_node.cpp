#include <cstdlib>
#include <iostream>
#include <boost/filesystem.hpp>
#include <string>
#include "ros/ros.h"
#include "MoveJStackAction.cpp"
#include "MoveJTrajStackAction.cpp"
#include "MoveJVelTrajStackAction.cpp"
#include <math.h>

using namespace std;

int main(int argc, char **argv)
{
	  ros::init(argc,argv, "iiwa_command_stack");
    ROS_INFO("Node registered as %s\n", ros::this_node::getName().c_str());

    ros::NodeHandle nh;

    MoveJStackAction movej("MoveJ");
    MoveJTrajStackAction movej_traj("MoveJTraj"); //Does not work because it interpolates to reach each position with v=0, so although you send a new position meanwhile the robot is constantly decreasing its maximum velocity
		MoveJVelTrajStackAction movejvel_traj("MoveJVelTraj");
	  bool success = true;
    ros::AsyncSpinner spinner(8);
    spinner.start();
    while (ros::ok())
    {
    }
	  return 0;
}
