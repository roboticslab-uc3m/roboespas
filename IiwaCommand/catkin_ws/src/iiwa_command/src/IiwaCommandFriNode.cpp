#ifndef IIWA_COMMAND_FRI_NODE
#define IIWA_COMMAND_FRI_NODE

#include <boost/bind.hpp>
#include "ros/ros.h"
#include "friLBRClient.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include <iiwa_command/MoveJTrajectoryAction.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <actionlib/server/simple_action_server.h>



using namespace KUKA::FRI;
using namespace std;

class IiwaCommandFriNode : public KUKA::FRI::LBRClient
{
protected:
  //MoveJ Trajectory Action Server
  ros::NodeHandle nh;
  actionlib::SimpleActionServer<iiwa_command::MoveJTrajectoryAction> as_movejtraj;
  iiwa_command::MoveJTrajectoryFeedback as_feedback;
  iiwa_command::MoveJTrajectoryResult as_result;

  //Parameters
  double control_step_size;
  Eigen::VectorXd qdot_max; //double qdot_max[7];
  Eigen::VectorXd q_max;  //double q_max[7];
  double qinc_max[7];
  double qinc_threshold = 1e-10;

  //Joint joint_positions
  double q_command[7];
  double q_read[7];
  double qtorque_read[7];
  double last_q_command[7];
  //double q_command_read[7];

  //Times
  ros::Publisher info_pub;

  //bool first_time;
  //int first_timestampSec;
  //int first_timestampNanosec;

  void callback_MoveJTrajectory(const iiwa_command::MoveJTrajectoryGoalConstPtr &goal)
  {
      ROS_INFO("MoveJTrajectory action server active");
      //Variables returned
      trajectory_msgs::JointTrajectory trajectory_commanded;
      vector<sensor_msgs::JointState> trajectory_read;
      //First save in an vector
      trajectory_msgs::JointTrajectory trajectory_desired = goal -> trajectory_desired;
      //Check position is not empty
      if (trajectory_desired.points.empty())
      {
          ROS_ERROR("Empty joint trajectory");
          as_movejtraj.setSucceeded(as_result);
          return;
      }
      //Variables returned
      int id = 0;
      //Time variables
      ros::Time tStartTraj = ros::Time::now();
      ros::Time currentTime = tStartTraj;
      while (id < trajectory_desired.points.size())
      {
          //Check which position should be commanded now
          currentTime = ros::Time::now();
          id = (currentTime-tStartTraj).toSec()/control_step_size;
          if (id>=trajectory_desired.points.size())
          {
            break;
          }
          //Set q_command variable
          vector<double> q_comm_vec = trajectory_desired.points[id].positions;
          copy(q_comm_vec.begin(), q_comm_vec.end(), q_command);

          //Save commanded point in trajectory_commanded
          trajectory_msgs::JointTrajectoryPoint point_command;
          for (int i=0; i<q_comm_vec.size(); i++)
          {
              point_command.positions.push_back(q_comm_vec[i]);
          }
          point_command.time_from_start = ros::Time::now()-tStartTraj;
          trajectory_commanded.points.push_back(point_command);

          //Save current q_read in trajectory_read
          vector<double> measured_joint_position(begin(q_read), end(q_read));
          vector<double> measured_joint_torque(begin(qtorque_read), end(qtorque_read));
          sensor_msgs::JointState joint_state;
        	for (int i = 0; i < 7; i++)
        	{
        		joint_state.position.push_back(measured_joint_position[i]);
        		joint_state.effort.push_back(measured_joint_torque[i]);
        	}
        	joint_state.name={"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
          /*
          LBRState currentRobotState=robotState();
          int timestampSec = currentRobotState.getTimestampSec();
          int timestampNanosec = currentRobotState.getTimestampNanoSec();
          if (first_time)
          {
            first_timestampSec=timestampSec;
            first_timestampNanosec=timestampNanosec;
            first_time=false;
          }
          int timestampSecFromStart=timestampSec-first_timestampSec;
          int timestampNanosecFromStart=timestampNanosec-first_timestampNanosec;
        	joint_state.header.stamp=ros::Time(timestampSecFromStart + 1e-9*timestampNanosecFromStart);
          */
          trajectory_read.push_back(joint_state);
      }
      as_result.trajectory_commanded = trajectory_commanded;
      as_result.trajectory_read = trajectory_read;
      as_movejtraj.setSucceeded(as_result);
      ROS_INFO("MoveJTrajectoryTrajectory action server result sent");
    }
public:
  IiwaCommandFriNode(string name): as_movejtraj(nh, name, boost::bind(&IiwaCommandFriNode::callback_MoveJTrajectory, this, _1), false)
  {
    //Read parameters
      //Control step size
    if (!nh.getParam("/iiwa_command/control_step_size", control_step_size))
    {
        ROS_ERROR("Failed to read '/iiwa_command/control_step_size' on param server");
    }
      //Q max
    vector<double> q_max_vec;
    if (!nh.getParam("/iiwa/limits/joint_position", q_max_vec))
    {
        ROS_ERROR("Failed to read '/iiwa/limits/joint_position' on param server");
    }
    q_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_max_vec.data(), q_max_vec.size());
    q_max = q_max*M_PI/180.0;
      //Qdot max
    vector<double> qdot_max_vec;
    if (!nh.getParam("/iiwa/limits/joint_velocity", qdot_max_vec))
    {
        ROS_ERROR("Failed to read '/iiwa/limits/joint_velocity' on param server");
    }
    qdot_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(qdot_max_vec.data(), qdot_max_vec.size());
    qdot_max = qdot_max*M_PI/180.0;
      //Qinc_max
    for (int j=0; j<7; j++)
    {
      qinc_max[j] = qdot_max[j]*control_step_size*M_PI/180.0; //Convert to radians and multiply by control_step_size
    }
    //publisher
    info_pub = nh.advertise<std_msgs::String>("/iiwa_fri/info", 1000);

    //Start action server
    as_movejtraj.start();
    ROS_INFO("Action server %s started", name.c_str());
  }
  virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState)
  {
      LBRClient::onStateChange(oldState, newState);
      // react on state change events
      switch (newState)
      {
         case MONITORING_WAIT:
         {
   		      ROS_INFO("Switched to MONITORING_WAIT\n");
            break;
         }
         case MONITORING_READY:
         {
   		      ROS_INFO("Switched to MONITORING_READY\n");
            break;
         }
         case COMMANDING_WAIT:
         {
           //Copy q_read into last_q_command and q_command
   		      //memcpy(last_q_command, q_read, LBRState::NUMBER_OF_JOINTS * sizeof(double));
   	  	    //memcpy(q_command, q_read, LBRState::NUMBER_OF_JOINTS * sizeof(double));
   		      ROS_INFO("Switched to COMMANDING_WAIT\n");
   		      break;
         }
         case COMMANDING_ACTIVE:
         {
       		  //Copy q_read into q_command
       		  //memcpy(q_command, q_read, LBRState::NUMBER_OF_JOINTS * sizeof(double));
       		  ROS_INFO("Switched to COMMANDING_ACTIVE\n");
            break;
         }
         default:
         {
            break;
         }
      }
  }
  virtual void monitor()
  {
    LBRClient::monitor();
  }
	virtual void waitForCommand()
  {
    LBRClient::waitForCommand();
    ROS_INFO("Waiting for Command");
  }
	virtual void command()
  {
    double diff[7];
    bool unreachable = false;
    bool steady = true;
    for (unsigned int i=0; i<7; i++)
    {
      diff[i] = abs(q_command[i] - last_q_command[i]);
      if (diff[i]-qinc_max[i] > qinc_threshold)
      {
        unreachable = true;
      }
      if (diff[i] > qinc_threshold)
      {
        cout << diff[i] << ", ";
        steady = false;
      }
    }
    if (steady == false)
    {
      cout << endl;
    }
    if (unreachable == true || steady == true)
    {
      std_msgs::String msg;
      stringstream ss;
      if (unreachable)
      {
        ss << "unreachable, ";
      }
      else if (steady)
      {
        ss << "steady, ";
      }
      ss << "qinc_max: " << qinc_max[0] << ", " << qinc_max[1] << ", " << qinc_max[2] << ", " << qinc_max[3] << ", " << qinc_max[4] << ", " << qinc_max[5] << ", " << qinc_max[6];
      msg.data = ss.str();
      info_pub.publish(msg);
    }
    else
    {
      memcpy(last_q_command, q_command, LBRState::NUMBER_OF_JOINTS*sizeof(double));
      std_msgs::String msg;
  		stringstream ss;
      	ss << "commanding, qinc_max: " << qinc_max[0] << ", " << qinc_max[1] << ", " << qinc_max[2] << ", " << qinc_max[3] << ", " << qinc_max[4] << ", " << qinc_max[5] << ", " << qinc_max[6] << endl;
  		ss << "diff: " << diff[0] << ", " << diff[1] << ", " << diff[2] << ", " << diff[3] << ", " << diff[4] << ", " << diff[5] << ", " << diff[6];
      	msg.data = ss.str();
  		info_pub.publish(msg);
    }
    robotCommand().setJointPosition(last_q_command);
  }
};

#endif
