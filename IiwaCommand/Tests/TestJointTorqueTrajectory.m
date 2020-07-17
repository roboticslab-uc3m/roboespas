%% Connect to ROS Network
clear all;
close all;
plot_feedback = 1;
plot_result = 1;
% Connect with ROS master
init_ros;
ptree=rosparam;
% Create actionlib to send a trajectory to gazebo iiwa robot, and some
% services to create and read the joint_trajectory messages faster
[iiwaCommandASrv_cli, iiwaCommandASrv_msg] = rosactionclient('iiwa_command');
iiwaCommandASrv_cli.ActivationFcn = @(~) disp('IiwaCommand action server active');
if (plot_feedback)
    figure; hold on;
    iiwaCommandASrv_cli.FeedbackFcn = @(~,msg) IiwaPlotter.joint_effort(msg.PointCommanded, msg.JointState.Effort, msg.TimeFromStart);
else
    iiwaCommandASrv_cli.FeedbackFcn = @(~,msg) (1);
end
iiwaCommandASrv_cli.ResultFcn = @(~,msg) disp('IiwaCommand action server result received');

[mdlConfig_cli, mdlConfig_msg] = rossvcclient('gazebo/set_model_configuration');

%% Create an LBR RigidBodyTree Object from URDF
control_step_size = get(ptree, '/iiwa_command/control_step_size');
lbr = importrobot('iiwa14.urdf');
lbr.DataFormat = 'row';
lbr.Gravity = [0 0 -9.80];
load lbr_waypoints.mat
traj_des=IiwaTrajectory(lbr, 'desired', tWaypoints, qWaypoints, 0:control_step_size:5);

%% Reset LBR to Home Configuration in Gazebo
% Fill action client desired trajectory
waitForServer(iiwaCommandASrv_cli);
iiwaCommandASrv_msg.TrajectoryDesired = IiwaMsgTransformer.toJointTraj(traj_des);

%Fill mdl config msg
mdlConfig_msg.ModelName = 'iiwa';
mdlConfig_msg.UrdfParamName = 'robot_description';
mdlConfig_msg.JointNames = {'iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3',...
                  'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7'};
mdlConfig_msg.JointPositions = homeConfiguration(lbr);

%% Move the robot
call(mdlConfig_cli, mdlConfig_msg);
resultMsg = sendGoalAndWait(iiwaCommandASrv_cli, iiwaCommandASrv_msg);

%% Inspect Results
% Transform into IiwaTrajectories
traj_comm = IiwaTrajectory('commanded', resultMsg.TrajectoryCommanded);
traj_output = IiwaTrajectory('output', resultMsg.TrajectoryJointState);

IiwaPlotter.joint_efforts({traj_comm, traj_output}, ['b', 'r']);
% Plot and inspect the actual joint torques and positions versus the desired values. Note that with the feed-forward torque,
% the PD torques should oscillate around zero.
IiwaPlotter.effortWithPD(traj_des, traj_comm);
IiwaPlotter.joint_positions({traj_comm, traj_output}, ['b', 'r']);
