#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <iiwa_command/MoveTorqueAction.h>
#include <actionlib/server/simple_action_server.h>
#include <Eigen/Dense>
#include <math.h>
#include "MoveTorqueAction.cpp"
#include "MoveJAction.cpp"
#include "MoveLAction.cpp"
#include "MoveLTrajectoryAction.cpp"
#include "IiwaScrewTheory.cpp"
#include "IiwaStatePublisher.cpp"
#include <math.h>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc,argv, "iiwa_command");
    ROS_INFO("Node registered as %s\n", ros::this_node::getName().c_str());

    ros::NodeHandle nh;
	//Load parameters from parameter server
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
	std::vector<double> qdotmax;
	if (!nh.getParam("/iiwa/limits/joint_velocity", qdotmax))
	{
		ROS_ERROR("Failed to read '/iiwa/limits/joint_velocity on param server'");
	}
	Eigen::VectorXd qdotMax = Eigen::Map<Eigen::Matrix<double, 1, 7>> (qdotmax.data());
	qdotMax = qdotMax*M_PI/180;
    IiwaScrewTheory::SetParameters(IiwaTwists, Hst0, qdotMax);
    MoveTorqueAction movetorque(ros::this_node::getName());
    MoveJAction movej("MoveJ");
    MoveLAction movel("MoveL");
	MoveLTrajectoryAction moveltrajectory("MoveLTrajectory");
	IiwaStatePublisher iiwastate("IiwaStatePublisher");

	bool success = true;
	while (ros::ok())
	{
		ros::spinOnce();
	}
	return 0;
}
