#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#define HOLD_FRAMES 100

//Based on Matlab tutorial https://es.mathworks.com/help/robotics/examples/control-lbr-manipulator-motion-through-joint-torque.html

namespace gazebo
{
    class ModelPush : public ModelPlugin
    {
        private: 
        ros::NodeHandle * nh;
        ros::Publisher joint_state_pub;
        ros::Subscriber joint_torque_sub;
        bool applyPastCommand;
        bool applyPastRead;
        int pastCommandCounter;
        int pastReadCounter;
        std::vector<float> pastCommandJointTorque;
        std::vector<float> pastCommandJointPosition;
        std::vector<float> pastReadJointPosition;
        int nFrame=1;

        public: void JointStateCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg)
        {
            this->applyPastCommand = true;
            this->pastCommandCounter = 0;
            this->pastCommandJointTorque.clear();
            this->pastCommandJointPosition.clear();
            //Add first empty torque bc there are 8 joints and first one is not the first of the robot, is the joint between the ground and the robot
            this->pastCommandJointTorque.push_back(0); 
            this->pastCommandJointPosition.push_back(0);
            for (int i = 0; i < msg->effort.size(); i++) 
            {
                this->pastCommandJointTorque.push_back(msg->effort[i]);
            }
            for (int i=0; i < msg->positions.size(); i++)
            {
                this->pastCommandJointPosition.push_back(msg->positions[i]);
            }
            ROS_INFO("joint_torque");
            for (int i=0; i<pastCommandJointTorque.size(); i++)
            {
                ROS_INFO("%f", pastCommandJointTorque[i]);
            }
            ROS_INFO("joint_position");
            for (int i=0; i<pastCommandJointPosition.size(); i++)
            {
                ROS_INFO("%f", pastCommandJointPosition[i]);
            }
            ROS_INFO("--");
        }

        public: 
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            // Store the pointer to the model
            this->model = _parent;

            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&ModelPush::OnUpdate, this, _1));
            this->nh = new ros::NodeHandle("iiwa_gazebo");
            this->joint_state_pub = nh->advertise<sensor_msgs::JointState>("joint_state", 1);
            this->joint_torque_sub = nh->subscribe("joint_command", 1, &ModelPush::JointStateCallback, this);
            this->pastCommandCounter = 0;
            this->applyPastCommand = false;

            ROS_INFO("Finished loading IIWA Gazebo Command Plugin.");
        }

        // Called by the world update start event
        public: 
        void OnUpdate(const common::UpdateInfo & _info)/*_info*/
        {
            std::string control_type;
            nh->getParam("/iiwa_gazebo_plugin/control_type", control_type);
            physics::Joint_V joints = this->model->GetJoints();
            //COMMAND ROBOT DEPENDING ON THE TYPE OF CONTROL
            if (strcmp(control_type.c_str(), "joint_torque")==0)
            {

                //Apply joint_torque received
                if (this->applyPastCommand)
                {
                    for (int i = 0; i < joints.size() && i < this->pastCommandJointTorque.size(); i++)
                    {
                        // Force is additive (multiple calls to SetForce to the same joint in the same time step 
                        // will accumulate forces on that Joint)?
                        joints[i]->SetForce(0, this->pastCommandJointTorque[i]);
                    }
                    this->pastCommandCounter += 1;
                    if (this->pastCommandCounter >= HOLD_FRAMES) 
                    { 
                        this->applyPastCommand = false;
                    }
                } 
            }
            else
            {
                //Apply joint_position received
                if (this->applyPastCommand)
                {
                    for (int i = 0; i < joints.size() && i < this->pastCommandJointPosition.size(); i++)
                    {
                        // Force is additive (multiple calls to SetForce to the same joint in the same time step 
                        // will accumulate forces on that Joint)?
                        joints[i]->SetPosition(0, this->pastCommandJointPosition[i]);
                    }
                    this->pastCommandCounter += 1;
                    if (this->pastCommandCounter >= HOLD_FRAMES) 
                    { 
                        this->applyPastCommand = false;
                    }
                }
                else
                {
                    if (this->applyPastRead)
                    {
                        for (int i=0; i<joints.size(); i++)
                        {
                            joints[i]->SetPosition(0, this->pastReadJointPosition[i]);
                        }
                        this->pastReadCounter += 1;
                        if (this->pastReadCounter >= HOLD_FRAMES) 
                        { 
                            this->applyPastRead = false;
                        }
                    }
                    else
                    {
                        this->pastReadCounter +=1;
                        if (this->pastReadCounter >= HOLD_FRAMES*2)
                        {
                            this->pastReadCounter = 0;
                            this->applyPastRead = true;
                        }
                    }
                }
                
            }



            //PUBLISH JOINT STATE AND UPDATE PASTREADJOINTPOSITION
            this->pastReadJointPosition.clear();
            sensor_msgs::JointState js_msg;
            std::vector<physics::JointWrench> wrenches;
            for (int i = 1; i < joints.size(); i++)
            {
                pastReadJointPosition.push_back(joints[i]->GetAngle(0).Radian());
                js_msg.position.push_back(joints[i]->GetAngle(0).Radian());
                js_msg.velocity.push_back(joints[i]->GetVelocity(0));
                wrenches.push_back(joints[i]->GetForceTorque(1));
            }
            //Torques in each joint, check the rotation direction for each joint at home position
		    js_msg.effort.push_back(-wrenches[0].body2Torque.z);
            js_msg.effort.push_back(wrenches[1].body1Torque.y);
            js_msg.effort.push_back(-wrenches[2].body2Torque.z);
            js_msg.effort.push_back(-wrenches[3].body1Torque.y);
		    js_msg.effort.push_back(-wrenches[4].body2Torque.z);
            js_msg.effort.push_back(wrenches[5].body1Torque.y);
            js_msg.effort.push_back(-wrenches[6].body2Torque.z);
            js_msg.name={"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
            js_msg.header.stamp=ros::Time::now();
            js_msg.header.seq=this->nFrame;
            this->nFrame = this->nFrame+1;
            this->joint_state_pub.publish(js_msg);
        }

        // Pointer to the model
        private: physics::ModelPtr model;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}

