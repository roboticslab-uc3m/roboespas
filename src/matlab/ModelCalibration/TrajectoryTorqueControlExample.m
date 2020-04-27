%% Removed comments
% In your MATLAB instance on the host computer, run the following commands 
% to initialize ROS global node in MATLAB and connect
% to the ROS master in the virtual machine (where Gazebo is running) through
% its IP address. Replace ipaddress with the IP address of your virtual machine.


% Gazebo provides two ROS services /gazebo/get_joint_properties and 
% /gazebo/apply_joint_effort that can be used to get joint state and set 
% joint torques. However, the services are too slow to close the torque control
% loop. Therefore, a customized Gazebo plug-in is used so that the joint 
% state/torques in Gazebo can be read/written at a much faster rate through
% the plain ROS topics (publisher and subscriber). The customized Gazebo 
% plug-in is already brought up together with Gazebo LBR Simulator.

% Pre-Compute Joint Torque Trajectory for Desired Motion
% Set the gravity to be the same as that in Gazebo.
% Load joint configuration waypoints. This gives the key frames for the 
% desired motion of the robot.

% sample_time is the planned control stepsize. We use it to populate a set of time 
% points where the trajectory needs to be evaluated and store it in vector
% tt.

% Generate desired motion trajectory for each joint.
% exampleHelperJointTrajectoryGeneration generates joint trajectories from 
% given ti.me and joint configuration waypoints. 
% The trajectories are generated using pchip so that the interpolated 
% joint position does not violate joint limits as long as the waypoints do not.

% Pre-compute feed-forward torques that ideally would realize the desired motion (assuming no disturbances or any kind of errors)
% using inverseDynamics. The following for loop takes some time to run. 
% To accelerate, consider used generated code for inverseDynamics. 
% See the last section for details on how to do it.

% Use Gazebo-provided service to reset the robot to its home configuration. 
% For details on how to work with ROS service in MATLAB, see Call and
% Provide ROS Services. 

% Compose the required service message. It includes the joint names and 
% corresponding joint positions to send to Gazebo. Call the service using 
% this message.
%% Connect to ROS Network
% Connect with ROS master
if (robotics.ros.internal.Global.isNodeActive)
    rosshutdown;
end
setenv('ROS_MASTER_URI', 'http://192.168.1.53:11311');
setenv('ROS_IP', '192.168.1.53');
rosinit;
ptree=rosparam;
% Create actionlib to send a trajectory to gazebo iiwa robot, and some
% services to create and read the joint_trajectory messages faster
[iiwaCommandASrv_cli, iiwaCommandASrv_msg] = rosactionclient('iiwa_command');
iiwaCommandASrv_cli.ActivationFcn = @(~) disp('IiwaCommand action server active');
iiwaCommandASrv_cli.FeedbackFcn = @(~,msg) (1); %Clean feedback function so it doesn't print the result every time it receives something
iiwaCommandASrv_cli.ResultFcn = @(~,msg) disp('IiwaCommand action server result received');

[mdlConfig_cli, mdlConfig_msg] = rossvcclient('gazebo/set_model_configuration');
plotter=IiwaPlotter();

%% Create an LBR RigidBodyTree Object from URDF
control_step_size = get(ptree, '/iiwa_command/control_step_size');
iiwa_robot = importrobot('iiwa14.urdf');
iiwa_robot.DataFormat = 'row';
iiwa_robot.Gravity = [0 0 -9.80];
load lbr_waypoints.mat
traj_des=IiwaTrajectory(iiwa_robot, 'desired', tWaypoints, qWaypoints, 0:control_step_size:5);

%% Reset LBR to Home Configuration in Gazebo
% Fill action client desired trajectory
waitForServer(iiwaCommandASrv_cli);
iiwaCommandASrv_msg.TrajectoryDesired = IiwaMsgTransformer.toJointTraj(traj_des);

%Fill mdl config msg
mdlConfig_msg.ModelName = 'iiwa';
mdlConfig_msg.UrdfParamName = 'robot_description';
mdlConfig_msg.JointNames = {'iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3',...
                  'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7'};
mdlConfig_msg.JointPositions = homeConfiguration(iiwa_robot);

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
