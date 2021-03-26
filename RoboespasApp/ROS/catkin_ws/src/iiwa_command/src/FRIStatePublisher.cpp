#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <eigen3/Eigen/Dense>
#include "FRIClient.cpp"
#include "iiwa_command/LBRStateMsg.h"
#include "friLBRClient.h"

using namespace std;
using namespace KUKA::FRI;

class FRIStatePublisher
{
  private:
    ros::NodeHandle nh;
    sensor_msgs::JointState joint_state;
    ros::Publisher js_pub;
    ros::Publisher FRIState_pub;
    FRIClient* friClient;

    //Fix state for each timer callback
    double q_read[7];
    double q_command_read[7];
    double q_torque_read[7];
    double q_torque_commanded[7];
    double q_torque_external[7];
    ros::Time curr_time_stamp;
    double control_step_size;
  public:
    FRIStatePublisher(std::string name, FRIClient* friClient_)
    {
        friClient = friClient_;
        js_pub = nh.advertise<sensor_msgs::JointState>("/iiwa_command/joint_state", 1000, false);
        FRIState_pub = nh.advertise<iiwa_command::LBRStateMsg>("/iiwa_command/FRIState", 1000);
        joint_state.name = {"J1","J2","J3","J4","J5","J6","J7"};
        ROS_INFO("Publisher /iiwa_command/joint_state started");
    }
    void PublishState()
    {
        control_step_size = friClient->control_step_size;
        nh.setParam("/iiwa_command/control_step_size", control_step_size);
        memcpy(q_read, friClient->q_read, LBRState::NUMBER_OF_JOINTS * sizeof(double));
        memcpy(q_torque_read, friClient->q_torque_read, LBRState::NUMBER_OF_JOINTS * sizeof(double));
        curr_time_stamp = friClient->curr_time_stamp;
        //Update joint_state
        joint_state.position.clear();
        for (int i=0; i<7; i++)
        {
            joint_state.position.push_back(q_read[i]);
        }
        joint_state.effort.clear();
        for (int i=0; i<7; i++)
        {
            joint_state.effort.push_back(q_torque_read[i]);
        }
        //Update stamp
        joint_state.header.stamp=curr_time_stamp;
        js_pub.publish(joint_state); //Just publish the whole joint_state when a new joint_position arrives
    }
    void PublishLBRState()
    {
      //LBR STATE
      iiwa_command::LBRStateMsg LBRState_msg;
      LBRState robot_state=friClient->robot_state;
      LBRState_msg.timestamp=friClient->curr_time_stamp.toSec();;
      LBRState_msg.sample_time = robot_state.getSampleTime();
      LBRState_msg.session_state = SESSION_STATES[robot_state.getSessionState()];
      LBRState_msg.connection_quality = CONNECTION_QUALITIES[robot_state.getConnectionQuality()];
      LBRState_msg.safety_state = SAFETY_STATES[robot_state.getSafetyState()];
      LBRState_msg.operation_mode = OPERATION_MODES[robot_state.getOperationMode()];
      LBRState_msg.drive_state = DRIVE_STATES[robot_state.getDriveState()];
      LBRState_msg.client_command_mode = CLIENT_COMMAND_MODES[robot_state.getClientCommandMode()];
      LBRState_msg.overlay_type = OVERLAY_TYPES[robot_state.getOverlayType()];
      LBRState_msg.control_mode = CONTROL_MODES[robot_state.getControlMode()];
      LBRState_msg.tracking_performance = robot_state.getTrackingPerformance();

      //Build measured joint position vector
      memcpy(q_read, robot_state.getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
      std::vector<double> measured_joint_position(begin(q_read), end(q_read));
      LBRState_msg.measured_joint_position = measured_joint_position;

      //Build commanded joint position vector
      memcpy(q_command_read, robot_state.getCommandedJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
      std::vector<double> commanded_joint_position(begin(q_command_read), end(q_command_read));
      LBRState_msg.commanded_joint_position = commanded_joint_position;

      //Build measured joint torque vector
      memcpy(q_torque_read, robot_state.getMeasuredTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
      std::vector<double> read_joint_torque(begin(q_torque_read), end(q_torque_read));
      LBRState_msg.measured_joint_torque = read_joint_torque;

      //Build commanded joint torque vector
      memcpy(q_torque_commanded, robot_state.getCommandedTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
      std::vector<double> commanded_joint_torque(begin(q_torque_commanded), end(q_torque_commanded));
      LBRState_msg.commanded_joint_torque = commanded_joint_torque;

      //Build external joint torque vector
      memcpy(q_torque_external, robot_state.getExternalTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
      std::vector<double> external_joint_torque(begin(q_torque_commanded), end(q_torque_commanded));
      LBRState_msg.external_joint_torque = external_joint_torque;

      //Fill interpolator joint position if not in Monitor Mode
      if (LBRState_msg.session_state!="MONITORING_READY" && LBRState_msg.session_state!="MONITORING_WAIT")
      {
        const double* interpolator_joint_position_pointer = robot_state.getIpoJointPosition(); //Interpolator commanded joint positions, not available in monitoring mode
        std::vector<double> interpolator_joint_position(interpolator_joint_position_pointer, interpolator_joint_position_pointer + 7);
        LBRState_msg.interpolator_joint_position = interpolator_joint_position;
      }
      else
      {
        std::vector<double> interpolator_joint_position(7, 0.0);
        LBRState_msg.interpolator_joint_position = interpolator_joint_position;
      }
      FRIState_pub.publish(LBRState_msg);
    }
};
