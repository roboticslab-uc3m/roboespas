#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iiwa_command/FromJointTrajectory.h>
#include <iiwa_command/ToJointTrajectory.h>
#include <iiwa_command/FromJointStateVec.h>
#include <iiwa_command/ToJointStateVec.h>
#include <iiwa_command/FromTwistStampedVec.h>
#include <iiwa_command/ToTwistStampedVec.h>


using namespace std;
class MsgTransformHelperNode
{
    private:
    ros::NodeHandle nh;
    ros::ServiceServer from_joint_traj_server;
    ros::ServiceServer to_joint_traj_server;
    ros::ServiceServer from_joint_state_vec_server;
    ros::ServiceServer to_joint_state_vec_server;
    ros::ServiceServer from_twist_stamped_vec_server;
    ros::ServiceServer to_twist_stamped_vec_server;
    public:
    MsgTransformHelperNode(std::string name)
    {
        ROS_INFO ("Node registered as %s\n", name.c_str());
        from_joint_traj_server = nh.advertiseService("/msg_transform_helper/from_joint_traj", &MsgTransformHelperNode::callback_from_joint_traj_server, this);
        to_joint_traj_server = nh.advertiseService("/msg_transform_helper/to_joint_traj", &MsgTransformHelperNode::callback_to_joint_traj_server, this);
        from_joint_state_vec_server = nh.advertiseService("/msg_transform_helper/from_joint_state_vec", &MsgTransformHelperNode::callback_from_joint_state_vec_server, this);
        to_joint_state_vec_server = nh.advertiseService("/msg_transform_helper/to_joint_state_vec", &MsgTransformHelperNode::callback_to_joint_state_vec_server, this);
        from_twist_stamped_vec_server = nh.advertiseService("/msg_transform_helper/from_twist_stamped_vec", &MsgTransformHelperNode::callback_from_twist_stamped_vec_server, this);
        to_twist_stamped_vec_server = nh.advertiseService("/msg_transform_helper/to_twist_stamped_vec", &MsgTransformHelperNode::callback_to_twist_stamped_vec_server, this);
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
        if (njoints!=0)
        {
            int npos=req.positions.size()/njoints;
            int nvels=req.velocities.size()/njoints;
            int naccs=req.accelerations.size()/njoints;
            int neffs=req.efforts.size()/njoints;
            int npoints = req.stamps.size();
            int it_positions=0;
            int it_velocities=0;
            int it_accelerations=0;
            int it_efforts=0;
            for (int i=0; i<req.stamps.size(); i++)
            {
                trajectory_msgs::JointTrajectoryPoint p;
                p.time_from_start=ros::Duration(req.stamps[i]);
                if (i<npos)
                {
                    for (int j=0; j<njoints; j++)
                    {
                        p.positions.push_back(req.positions[it_positions]);
                        it_positions=it_positions+1;
                    }
                }
                if (i<nvels)
                {
                    for (int j=0; j<njoints; j++)
                    {
                        p.velocities.push_back(req.velocities[it_velocities]);
                        it_velocities=it_velocities+1;
                    }
                }
                if (i<naccs)
                {
                    for (int j=0; j<njoints; j++)
                    {
                        p.accelerations.push_back(req.accelerations[it_accelerations]);
                        it_accelerations=it_accelerations+1;
                    }
                }
                if (i<neffs)
                {
                    for (int j=0; j<njoints; j++)
                    {
                        p.effort.push_back(req.efforts[it_efforts]);
                        it_efforts=it_efforts+1;
                    }
                }
                res.joint_trajectory.points.push_back(p);
            }
        }
        else
        {
            ROS_ERROR("ToJointTrajectoryServer Error: Fill joint_names first");
            return false;
        }
        return true;
    }
    bool callback_from_joint_state_vec_server(iiwa_command::FromJointStateVec::Request &req, iiwa_command::FromJointStateVec::Response &res)
    {
        if (req.joint_state_vec.size()>0)
        {
            double first_stamp=req.joint_state_vec[0].header.stamp.toSec();
            res.joint_names=req.joint_state_vec[0].name;
            for (int i=0; i<req.joint_state_vec.size(); i++)
            {
                res.stamps.push_back(req.joint_state_vec[i].header.stamp.toSec()-first_stamp);
                for (int j=0; j<req.joint_state_vec[i].position.size(); j++)
                    res.positions.push_back(req.joint_state_vec[i].position[j]);
                for (int j=0; j<req.joint_state_vec[i].velocity.size(); j++)
                    res.velocities.push_back(req.joint_state_vec[i].velocity[j]);
                for (int j=0; j<req.joint_state_vec[i].effort.size(); j++)
                    res.efforts.push_back(req.joint_state_vec[i].effort[j]);
            }
        }
        return true;
    }
    bool callback_to_joint_state_vec_server(iiwa_command::ToJointStateVec::Request &req, iiwa_command::ToJointStateVec::Response &res)
    {
        ROS_ERROR("Not implemented yet");
        //TODO: Implement
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
        return true;
    }
    bool callback_from_twist_stamped_vec_server(iiwa_command::FromTwistStampedVec::Request &req, iiwa_command::FromTwistStampedVec::Response &res)
    {
        if (req.twist_stamped_vec.size()>0)
        {
            double first_stamp=req.twist_stamped_vec[0].header.stamp.toSec();
            for (int i=0; i<req.twist_stamped_vec.size(); i++)
            {
                res.stamps.push_back(req.twist_stamped_vec[i].header.stamp.toSec()-first_stamp);
                res.values.push_back(req.twist_stamped_vec[i].twist.linear.x);
                res.values.push_back(req.twist_stamped_vec[i].twist.linear.y);
                res.values.push_back(req.twist_stamped_vec[i].twist.linear.z);
                res.values.push_back(req.twist_stamped_vec[i].twist.angular.x);
                res.values.push_back(req.twist_stamped_vec[i].twist.angular.y);
                res.values.push_back(req.twist_stamped_vec[i].twist.angular.z);
            }
        }
        return true;
    }
    bool callback_to_twist_stamped_vec_server(iiwa_command::ToTwistStampedVec::Request &req, iiwa_command::ToTwistStampedVec::Response &res)
    {
        vector<geometry_msgs::TwistStamped> twist_stamped_vec;
        int npoints = req.stamps.size();
        int id_values = 0 ;
        ros::Time time_start = ros::Time::now();
        for (unsigned int i=0; i<npoints; i++)
        {
            geometry_msgs::TwistStamped twist_stamped;
            twist_stamped.twist.linear.x=req.values[id_values];
            id_values++;
            twist_stamped.twist.linear.y=req.values[id_values];
            id_values++;
            twist_stamped.twist.linear.z=req.values[id_values];
            id_values++;
            twist_stamped.twist.angular.x=req.values[id_values];
            id_values++;
            twist_stamped.twist.angular.y=req.values[id_values];
            id_values++;
            twist_stamped.twist.angular.z=req.values[id_values];
            id_values++;
            twist_stamped.header.stamp = ros::Time(req.stamps[i]);
            twist_stamped_vec.push_back(twist_stamped);
        }
        /*if (req.twist_stamped_vec.size()>0)
        {
            double first_stamp=req.twist_stamped_vec[0].header.stamp.toSec();
            for (int i=0; i<req.twist_stamped_vec.size(); i++)
            {
                res.stamps.push_back(req.twist_stamped_vec[i].header.stamp.toSec()-first_stamp);
                res.values.push_back(req.twist_stamped_vec[i].twist.linear.x);
                res.values.push_back(req.twist_stamped_vec[i].twist.linear.y);
                res.values.push_back(req.twist_stamped_vec[i].twist.linear.z);
                res.values.push_back(req.twist_stamped_vec[i].twist.angular.x);
                res.values.push_back(req.twist_stamped_vec[i].twist.angular.y);
                res.values.push_back(req.twist_stamped_vec[i].twist.angular.z);
            }
        }*/
        //TODO
        res.twist_stamped_vec = twist_stamped_vec;
        return true;
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
