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
plotter=IiwaPlotter();

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

%% Move the robot while plotting the feedback
call(mdlConfig_cli, mdlConfig_msg);
figure;
hold on;

% Comment this to stop plot while executing
iiwaCommandASrv_cli.FeedbackFcn=@(~, msg) plotter.effortPoint(msg);
resultMsg = sendGoalAndWait(iiwaCommandASrv_cli, iiwaCommandASrv_msg);

% Transform into IiwaTrajectories
traj_comm = IiwaTrajectory('commanded', resultMsg.TrajectoryCommanded);
traj_output = IiwaTrajectory('output', resultMsg.TrajectoryJointState);

%% Inspect Results

plotter.joint_efforts(traj_comm, traj_output);
% Plot and inspect the actual joint torques and positions versus the desired values. Note that with the feed-forward torque,
% the PD torques should oscillate around zero.
plotter.effortWithPD(traj_des, traj_comm);
plotter.joint_positions(traj_comm, traj_output);
