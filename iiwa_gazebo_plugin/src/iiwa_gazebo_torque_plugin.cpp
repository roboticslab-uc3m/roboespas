#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "iiwa_msgs/JointPositionStamped.h"
#include "iiwa_msgs/JointVelocityStamped.h"
#include "iiwa_msgs/JointTorqueStamped.h"
#include "iiwa_msgs/WrenchStamped.h"

#define HOLD_FRAMES 100

namespace gazebo
{
    class ModelPush : public ModelPlugin
    {
        private: 
        ros::NodeHandle * n;
        ros::Publisher joint_position_pub;
        ros::Publisher joint_velocity_pub;
        ros::Publisher joint_torque_pub;
        ros::Subscriber joint_torque_sub;
        bool applyPastCommand;
        int pastCommandCounter;
        std::vector<float> pastCommand;

        public: void JointTorqueCallback(const iiwa_msgs::JointTorqueStamped::ConstPtr& msg)
        {
            this->applyPastCommand = true;
            this->pastCommandCounter = 0;
            this->pastCommand.clear();

            this->pastCommand.push_back(0);
            for (int i = 0; i < msg->joint_torque.size(); i++) 
                this->pastCommand.push_back(msg->joint_torque[i]);
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
            this->n = new ros::NodeHandle("iiwa_gazebo_plugin");
            this->joint_position_pub = n->advertise<iiwa_msgs::JointPositionStamped>("iiwa_state/joint_position", 1);
            this->joint_velocity_pub = n->advertise<iiwa_msgs::JointVelocityStamped>("iiwa_state/joint_velocity", 1);
            this->joint_torque_pub = n->advertise<iiwa_msgs::JointTorqueStamped>("iiwa_state/joint_torque", 1);
            this->joint_torque_sub = n->subscribe("iiwa_command/joint_torque", 1, &ModelPush::JointTorqueCallback, this);
            this->pastCommandCounter = 0;
            this->applyPastCommand = false;

            ROS_INFO("Finished loading IIWA Torque Command Plugin.");
        }

        // Called by the world update start event
        public: 
        void OnUpdate(const common::UpdateInfo & _info)/*_info*/
        {
            iiwa_msgs::JointPositionStamped jp_msg;
            iiwa_msgs::JointVelocityStamped jv_msg;
            iiwa_msgs::JointTorqueStamped jt_msg;

            physics::Joint_V joints = this->model->GetJoints();

            if (this->applyPastCommand)
            {
                for (int i = 0; i < joints.size() && i < this->pastCommand.size(); i++)
                {
                    // Force is additive (multiple calls to SetForce to the same joint in the same time step 
                    // will accumulate forces on that Joint)?
                    joints[i]->SetForce(0, this->pastCommand[i]);
                }
                this->pastCommandCounter += 1;
                if (this->pastCommandCounter >= HOLD_FRAMES) 
                { 
                    this->applyPastCommand = false;
                }
            }
            std::vector<physics::JointWrench> wrenches;
            for (int i = 1; i < joints.size(); i++)
            {
                jp_msg.joint_position.push_back(joints[i]->GetAngle(0).Radian());
                jv_msg.joint_velocity.push_back(joints[i]->GetVelocity(0));
                wrenches.push_back(joints[i]->GetForceTorque(1));
            }

            jt_msg.joint_torque.push_back(-wrenches[0].body2Torque.z);
            jt_msg.joint_torque.push_back(-wrenches[1].body2Torque.y);
            jt_msg.joint_torque.push_back(-wrenches[2].body2Torque.z);
            jt_msg.joint_torque.push_back(wrenches[3].body2Torque.y);
            jt_msg.joint_torque.push_back(-wrenches[4].body2Torque.z);
            jt_msg.joint_torque.push_back(-wrenches[5].body2Torque.y);
            jt_msg.joint_torque.push_back(-wrenches[6].body2Torque.z);

            jp_msg.stamp=_info.simTime.Float();
            jv_msg.stamp=_info.simTime.Float();
            jt_msg.stamp=_info.simTime.Float();
            this->joint_position_pub.publish(jp_msg);
            this->joint_velocity_pub.publish(jv_msg);
            this->joint_torque_pub.publish(jt_msg);

        }

        // Pointer to the model
        private: physics::ModelPtr model;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}

