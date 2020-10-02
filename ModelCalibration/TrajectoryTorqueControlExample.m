%% Connect to ROS Network
% Connect with ROS master
init_ros;
ptree=rosparam;
% Create actionlib to send a trajectory to gazebo iiwa robot, and some
% services to create and read the joint_trajectory messages faster
[iiwaCommandASrv_cli, iiwaCommandASrv_msg] = rosactionclient('MoveTorque');
iiwaCommandASrv_cli.ActivationFcn = @(~) disp('MoveTorque action server active');
iiwaCommandASrv_cli.FeedbackFcn = @(~,msg) (1); %Clean feedback function so it doesn't print the result every time it receives something
iiwaCommandASrv_cli.ResultFcn = @(~,msg) disp('MoveTorque action server result received');
[mdlConfig_cli, mdlConfig_msg] = rossvcclient('gazebo/set_model_configuration');

%% Create an LBR RigidBodyTree Object from URDF
%load('/home/roboespas/roboespas/IDSimulation/input_traj.mat'); %loads traj_des
traj_des_q = traj_des.CompleteJoint([0 0 0 0 0 0 0]);


control_step_size = get(ptree, '/iiwa_command/control_step_size');
lbr = importrobot('iiwa14.urdf');
lbr.DataFormat = 'row';
lbr.Gravity = [0 0 -9.80];

traj_spline = IiwaTrajectoryGeneration.BoundedSplineTrajectory(traj_des_q, 10e-4, 100);
traj_des_st = traj_spline.CompleteEffort('st');
traj_des_m = traj_spline.CompleteEffort('matlab');

%% Reset LBR to Home Configuration in Gazebo
% Fill action client desired trajectory
clear IiwaMsgTransformer
waitForServer(iiwaCommandASrv_cli);
iiwaCommandASrv_msg.TrajectoryDesired = IiwaMsgTransformer.toJointTraj(traj_des_m);

%Fill mdl config msg
mdlConfig_msg.ModelName = 'iiwa';
mdlConfig_msg.UrdfParamName = 'robot_description';
mdlConfig_msg.JointNames = {'iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3',...
                  'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7'};
mdlConfig_msg.JointPositions = homeConfiguration(lbr);

% Move the robot while plotting the feedback
call(mdlConfig_cli, mdlConfig_msg);

% Comment this to stop plot while executing
resultMsg = sendGoalAndWait(iiwaCommandASrv_cli, iiwaCommandASrv_msg);

% Transform into IiwaTrajectories
traj_comm = IiwaTrajectory('commanded', resultMsg.TrajectoryCommanded);
traj_output = IiwaTrajectory('output', resultMsg.TrajectoryJointState);

%% Inspect Results

IiwaPlotter.joint_efforts({traj_des_st, traj_output}, ['b', 'r']);
% Plot and inspect the actual joint torques and positions versus the desired values. Note that with the feed-forward torque,
% the PD torques should oscillate around zero.
%IiwaPlotter.effortWithPD(traj_spline, traj_comm);
IiwaPlotter.joint_positions({traj_des_st, traj_output}, ['m', 'b', 'r']);
IiwaPlotter.cartesian_positions({traj_des_st, traj_output}, ['m', 'b', 'r']);

IiwaPlotter.joint_position_error(traj_comm, traj_output, 'r');
IiwaPlotter.cartesian_position_error(traj_des_m, traj_output, 'r');
