#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <Eigen/Dense>
#include <math.h>
#include <iiwa_command/FromJointTrajectory.h>
#include <iiwa_command/ToJointTrajectory.h>
#include <iiwa_command/FromJointStateVec.h>
#include <iiwa_command/ToJointStateVec.h>

class MsgTransformHelperNode 
{
    private:
    ros::NodeHandle nh;
    ros::ServiceServer from_joint_traj_server;
    
    public:
    MsgTransformHelperNode(std::string name)
    {
        ROS_INFO ("Node registered as %s\n", name.c_str());
        from_joint_traj_server = nh.advertiseService("/msg_transform_helper/from_joint_traj", &MsgTransformHelperNode::callback_from_joint_traj_server, this);
	}
    bool callback_from_joint_traj_server(iiwa_command::FromJointTrajectory::Request &req, iiwa_command::FromJointTrajectory::Response &res)
    {
        res.joint_names=req.joint_trajectory.joint_names;
        for (int i=0; i<req.joint_trajectory.points.size(); i++)
        {
            res.stamps.push_back(req.joint_trajectory.points[i].time_from_start.toSec());
            for (int j=0; j<req.joint_trajectory.points[i].positions.size(); j++)
                res.positions.push_back(req.joint_trajectory.points[i].positions[j]);
            for (int j=0; j<req.joint_trajectory.points[i].velocities.size(); j++)
                res.velocities.push_back(req.joint_trajectory.points[i].velocities[j]);
            for (int j=0; j<req.joint_trajectory.points[i].accelerations.size(); j++)
                res.accelerations.push_back(req.joint_trajectory.points[i].accelerations[j]);
            for (int j=0; j<req.joint_trajectory.points[i].effort.size(); j++)
                res.efforts.push_back(req.joint_trajectory.points[i].effort[j]);
        }
        return true;
    }
    bool callback_to_joint_traj_server(iiwa_command::ToJointTrajectory::Request &req, iiwa_command::ToJointTrajectory::Response &res)
    {
        res.joint_trajectory.joint_names=req.joint_names;
        int njoints=res.joint_trajectory.joint_names.size();
        for (int i=0; i<req.stamps.size(); i++)
        {
            res.joint_trajectory.points[i].time_from_start= ros::Duration(req.stamps[i]);
        }
        int n=0;
        for (int i=0; i<(req.positions.size()/njoints); i++)
        {
            for (int j=0; j<njoints; j++)
            {
                res.joint_trajectory.points[i].positions.push_back(req.positions[n]);
                n=n+1;
            }
        }
        n=0;
        for (int i=0; i<(req.velocities.size()/njoints); i++)
        {
            for (int j=0; j<njoints; j++)
            {
                res.joint_trajectory.points[i].velocities.push_back(req.velocities[n]);
                n=n+1;
            }
        }
        n=0;
        for (int i=0; i<(req.accelerations.size()/njoints); i++)
        {
            for (int j=0; j<njoints; j++)
            {
                res.joint_trajectory.points[i].accelerations.push_back(req.accelerations[n]);
                n=n+1;
            }
        }
        n=0;
        for (int i=0; i<(req.efforts.size()/njoints); i++)
        {
            for (int j=0; j<njoints; j++)
            {
                res.joint_trajectory.points[i].effort.push_back(req.efforts[n]);
                n=n+1;
            }
        }
        return true;
    }
    bool callback_from_joint_state_vec_server(iiwa_command::FromJointStateVec::Request &req, iiwa_command::FromJointStateVec::Response &res)
    {
        /*
        res.joint_names=req.joint_trajectory.joint_names;
        for (int i=0; i<req.joint_trajectory.points.size(); i++)
        {
            res.stamps.push_back(req.joint_trajectory.points[i].time_from_start.toSec());
            for (int j=0; j<req.joint_trajectory.points[i].positions.size(); j++)
                res.positions.push_back(req.joint_trajectory.points[i].positions[j]);
            for (int j=0; j<req.joint_trajectory.points[i].velocities.size(); j++)
                res.velocities.push_back(req.joint_trajectory.points[i].velocities[j]);
            for (int j=0; j<req.joint_trajectory.points[i].accelerations.size(); j++)
                res.accelerations.push_back(req.joint_trajectory.points[i].accelerations[j]);
            for (int j=0; j<req.joint_trajectory.points[i].effort.size(); j++)
                res.efforts.push_back(req.joint_trajectory.points[i].effort[j]);
        }
        return true;
        */
    }
    bool callback_to_joint_state_vec_server(iiwa_command::ToJointStateVec::Request &req, iiwa_command::ToJointStateVec::Response &res)
    {
        /*
        res.joint_names=req.joint_trajectory.joint_names;
        for (int i=0; i<req.joint_trajectory.points.size(); i++)
        {
            res.stamps.push_back(req.joint_trajectory.points[i].time_from_start.toSec());
            for (int j=0; j<req.joint_trajectory.points[i].positions.size(); j++)
                res.positions.push_back(req.joint_trajectory.points[i].positions[j]);
            for (int j=0; j<req.joint_trajectory.points[i].velocities.size(); j++)
                res.velocities.push_back(req.joint_trajectory.points[i].velocities[j]);
            for (int j=0; j<req.joint_trajectory.points[i].accelerations.size(); j++)
                res.accelerations.push_back(req.joint_trajectory.points[i].accelerations[j]);
            for (int j=0; j<req.joint_trajectory.points[i].effort.size(); j++)
                res.efforts.push_back(req.joint_trajectory.points[i].effort[j]);
        }
        return true;
        */
    }
};

int main(int argc, char **argv)
{
	ros::init(argc,argv, "msg_transform_helper");
	
	MsgTransformHelperNode msg_transform_node=MsgTransformHelperNode(ros::this_node::getName());

	
	bool success = true;
	while (ros::ok())
	{
		ros::spinOnce();
	}
	return 0;
}
