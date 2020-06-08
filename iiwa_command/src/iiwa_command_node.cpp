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
#include "MoveLAction.cpp"
#include "IiwaScrewTheory.cpp"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc,argv, "iiwa_command");
    ROS_INFO("Node registered as %s\n", ros::this_node::getName().c_str());

    
    ros::NodeHandle nh;
    std::vector<double> twists;
    if (!nh.getParam("/iiwa/twists", twists))
    {
        ROS_ERROR("Failed to read '/iiwa/twists' on param server");
    }
    Eigen::MatrixXd IiwaTwists = Eigen::Map<Eigen::Matrix<double, 7, 6>>(twists.data()).transpose();
    std::vector<double> hst0;
    if (!nh.getParam("/iiwa/Hst0", hst0))
    {
        ROS_ERROR("Failed to read '/iiwa/Hst0' on param server");
    }
    Eigen::Matrix4d Hst0 = Eigen::Map<Eigen::Matrix<double, 4,4>>(hst0.data()).transpose();

    IiwaScrewTheory::SetParameters(IiwaTwists, Hst0);
    IiwaCommandAction iiwacommand(ros::this_node::getName());
    MoveJAction movej("MoveJ");
    MoveLAction movel("MoveL");

	bool success = true;
	while (ros::ok())
	{
		ros::spinOnce();
	}
	return 0;
}
