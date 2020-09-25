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

  //Iiwa state pub
  ros::Publisher joint_state_pub;
  ros::Publisher info_pub;
  sensor_msgs::JointState joint_state_current;

  //Parameters
  double control_step_size;
  Eigen::VectorXd qdot_max; //double qdot_max[7];
  Eigen::VectorXd q_max;  //double q_max[7];
  Eigen::VectorXd last_q_command;
  Eigen::VectorXd qinc_max;
  double qinc_threshold = 1e-10;
  double v_percentage;

  //Joint joint_positions
  double q_command[7];
  double q_command_read[7];
  double q_read[7];
  double qtorque_read[7];
  //Times
  bool first_time;
	int first_timestampSec;
	int first_timestampNanosec;

  void callback_MoveJTrajectory(const iiwa_command::MoveJTrajectoryGoalConstPtr &goal)
  {
      ROS_INFO("MoveJTrajectory action server active");
      //Read control_step_size from parameter server
      if (!nh.getParam("/iiwa_command/control_step_size", control_step_size))
      {
        ROS_ERROR("Failed to read '/iiwa_command/control_step_size' on param server");
      }
      //Read qdot_max
      vector<double> qdot_max_vec;
      if (!nh.getParam("/iiwa/limits/joint_velocity", qdot_max_vec))
      {
          ROS_ERROR("Failed to read '/iiwa/limits/joint_velocity' on param server");
      }
      qdot_max = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(qdot_max_vec.data(), qdot_max_vec.size());
      qdot_max = qdot_max*M_PI/180.0;
      //V percentage
      if (!nh.getParam("/iiwa_command/velocity", v_percentage))
      {
          ROS_ERROR("Failed to read '/iiwa_command/velocity' on param server");
      }
      //Calculate qinc_max with current maximum velocity
      qinc_max = qdot_max*v_percentage*control_step_size*1.5; //Convert to radians and multiply by control_step_size
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
      ros::Time currentTime;
      ros::Duration sleepDur;
      ros::Time contTime;
      ros::Duration proc_time;
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
          for (int i=0; i<7; i++)
          {
              point_command.positions.push_back(q_command[i]);
          }
          point_command.time_from_start = currentTime-tStartTraj;
          trajectory_commanded.points.push_back(point_command);
          joint_state_current.header.stamp = ros::Time((currentTime-tStartTraj).toSec());
          trajectory_read.push_back(joint_state_current);
          contTime = tStartTraj + ros::Duration(control_step_size*(id+1));
          sleepDur = contTime-ros::Time::now();
          //proc_time = ros::Time::now()-currentTime;
          //sleepDur = ros::Duration(control_step_size-proc_time.toSec());
          sleepDur.sleep();
      }
      //Seguir capturando 1 segundo más
      int times = 1/control_step_size;
      int t_i=0;
      while (t_i<times)
      {
        currentTime = ros::Time::now();
        joint_state_current.header.stamp = ros::Time((currentTime-tStartTraj).toSec());
        trajectory_read.push_back(joint_state_current);
        t_i++;
        id++;
        contTime = tStartTraj + ros::Duration(control_step_size*(id+1));
        sleepDur = contTime-ros::Time::now();
        sleepDur.sleep();
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
    qinc_max = qdot_max*v_percentage*control_step_size*1.5; //Convert to radians and multiply by control_step_size
    //Initialize times
    first_time=true;
    first_timestampSec=0;
    first_timestampNanosec=0;
    //Publisher
    info_pub = nh.advertise<std_msgs::String>("/iiwa_fri/info", 1000);
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("/iiwa_command/joint_state", 1000);
    //Start action server
    as_movejtraj.start();
    ROS_INFO("Action server %s started", name.c_str());
  }
  void updateJointState()
  {

    LBRState currentRobotState=robotState();
    memcpy(q_read, currentRobotState.getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    memcpy(q_command_read, currentRobotState.getCommandedJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    memcpy(qtorque_read, currentRobotState.getMeasuredTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    //Sample time
    control_step_size = currentRobotState.getSampleTime();;
    nh.setParam("/iiwa_command/control_step_size", control_step_size);
    //Calculate timestamp
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

    //Build joint_state
    joint_state_current.position.clear();
    joint_state_current.effort.clear();
    for (int i = 0; i < 7; i++)
    {
      joint_state_current.position.push_back(q_read[i]);
      joint_state_current.effort.push_back(qtorque_read[i]);
    }
    joint_state_current.name={"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
    joint_state_current.header.stamp=ros::Time(timestampSecFromStart + 1e-9*timestampNanosecFromStart);
    joint_state_pub.publish(joint_state_current);
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
   	  	    memcpy(q_command, q_read, LBRState::NUMBER_OF_JOINTS * sizeof(double));
            last_q_command = Eigen::Map<Eigen::VectorXd>(q_command, 7);
   		      ROS_INFO("Switched to COMMANDING_WAIT\n");
   		      break;
         }
         case COMMANDING_ACTIVE:
         {
       		  memcpy(q_command, q_read, LBRState::NUMBER_OF_JOINTS * sizeof(double));
            last_q_command = Eigen::Map<Eigen::VectorXd>(q_command, 7);
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
    this->updateJointState();
  }
	virtual void waitForCommand()
  {
    LBRClient::waitForCommand();
    ROS_INFO("Waiting for Command");
  }

	virtual void command()
  {
    this->updateJointState();

    Eigen::VectorXd q_command_ = Eigen::Map<Eigen::VectorXd>(q_command, 7);
    Eigen::VectorXd q_read_ = Eigen::Map<Eigen::VectorXd>(q_read, 7);
    Eigen::VectorXd qinc_command = (q_command_ - last_q_command).cwiseAbs();
    bool unreachable = false;
    for (int i=0; i<7; i++)
    {
      if (qinc_command[i]>qinc_max[i])
      {
        unreachable=true;
      }
    }
    for (int i=0; i<7; i++)
    {
      if (q_command_[i]>q_max[i])
      {
        unreachable=true;
      }
    }
    bool steady = true;
    for (int i=0; i<7; i++)
    {
      if (qinc_command[i]>qinc_threshold)
      {
        steady=false;
      }
    }
    std_msgs::String msg;
    stringstream ss;
    if (unreachable)
    {
      ss << "unreachable, qinc_comm: " << qinc_command.transpose() << ", qinc_max: " << qinc_max.transpose();
      msg.data = ss.str();
      info_pub.publish(msg);
      //memcpy(q_command, last_q_command.data(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    }
    else if (steady)
    {
      //ss << "steady, qinc_comm: " << qinc_command.transpose() << ", qinc_threshold: " << qinc_threshold;
      //msg.data = ss.str();
      //info_pub.publish(msg);
      memcpy(q_command, last_q_command.data(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
    }
    else
    {
      ss << "moving, qinc_comm: " << qinc_command.transpose();
      msg.data = ss.str();
      //info_pub.publish(msg);
    }
    robotCommand().setJointPosition(q_command); //last_q_command);  //Always necessary
    q_command_ = Eigen::Map<Eigen::VectorXd>(q_command, 7);
    last_q_command = q_command_;
  }
};

#endif
