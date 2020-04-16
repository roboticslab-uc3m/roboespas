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
% given time and joint configuration waypoints. 
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
%% Connect to ROS Network from MATLAB&reg;
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
iiwaCommandASrv_cli.FeedbackFcn = @(~,msg) disp('IiwaCommand feedback received');
iiwaCommandASrv_cli.ResultFcn = @(~,msg) disp('IiwaCommand action server result received');

[fromJointTrajSrv_cli, fromJointTrajSrv_msg] = rossvcclient('/msg_transform_helper/from_joint_traj');
[toJointTrajSrv_cli, toJointTrajSrv_msg] = rossvcclient('/msg_transform_helper/to_joint_traj');
[fromJointStateVecSrv_cli, fromJointStateVecSrv_msg] = rossvcclient('/msg_transform_helper/from_joint_state_vec');
[toJointStateVecSrv_cli, toJointStateVecSrv_msg] = rossvcclient('/msg_transform_helper/to_joint_state_vec');
[mdlConfig_cli, mdlConfig_msg] = rossvcclient('gazebo/set_model_configuration');

%% Create an LBR RigidBodyTree Object from URDF
lbr = importrobot('iiwa14.urdf');
lbr.DataFormat = 'row';
lbr.Gravity = [0 0 -9.80];
sample_time = get(ptree, '/iiwa_command/sample_time');
tt = 0:sample_time:5;

load lbr_waypoints.mat

[qDesired, qdotDesired, qddotDesired, tt] = exampleHelperJointTrajectoryGeneration(tWaypoints, qWaypoints, tt);
sample_time_new=tt(2)-tt(1);
if (abs(max(tt(2:end)-tt(1:end-1))-sample_time)>sample_time)
    disp('error with tt')
end
tauDesired = zeros(size(qDesired,1),7);
for i = 1:size(qDesired,1)
    tauDesired(i,:) = inverseDynamics(lbr, qDesired(i,:), qdotDesired(i,:), qddotDesired(i,:));
end

%% Reset LBR to Home Configuration in Gazebo
%Build joint trajectory msg
toJointTrajSrv_msg.JointNames={'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'};
toJointTrajSrv_msg.Efforts=reshape(tauDesired', size(tauDesired,1)*size(tauDesired,2), 1);
toJointTrajSrv_msg.Positions=reshape(qDesired', size(qDesired,1)*size(qDesired,2), 1);
toJointTrajSrv_msg.Velocities=reshape(qdotDesired', size(qdotDesired,1)*size(qdotDesired,2), 1);
toJointTrajSrv_msg.Accelerations=reshape(qddotDesired', size(qddotDesired,1)*size(qddotDesired,2), 1);
toJointTrajSrv_msg.Stamps=tt;
jointTrajMsg=toJointTrajSrv_cli.call(toJointTrajSrv_msg);

% Fill action client desired trajectory
waitForServer(iiwaCommandASrv_cli);
iiwaCommandASrv_msg.TrajectoryDesired = jointTrajMsg.JointTrajectory;

%Fill mdl config msg
mdlConfig_msg.ModelName = 'iiwa';
mdlConfig_msg.UrdfParamName = 'robot_description';
mdlConfig_msg.JointNames = {'iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3',...
                  'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7'};
mdlConfig_msg.JointPositions = homeConfiguration(lbr);

%% Move the robot
call(mdlConfig_cli, mdlConfig_msg);
resultMsg = sendGoalAndWait(iiwaCommandASrv_cli, iiwaCommandASrv_msg);

%% Compare torques
% Transform into vectors
fromJointTrajSrv_msg.JointTrajectory=resultMsg.TrajectoryCommanded;
traj_comm=fromJointTrajSrv_cli.call(fromJointTrajSrv_msg);
fromJointStateVecSrv_msg.JointStateVec=resultMsg.TrajectoryJointState;
traj_read=fromJointStateVecSrv_cli.call(fromJointStateVecSrv_msg);

data_comm=toDataStruct(traj_comm);
data_read=toDataStruct(traj_read);
%%
figure;
for i=1:7
    subplot(7,1,i);
    plot(data_comm.effort(i,:));
    hold on;
    plot(data_read.effort(i,:));
    legend('commanded', 'read');
end 

%% Inspect Results
% Plot and inspect the actual joint torques and positions versus the desired values. Note that with the feed-forward torque,
% the PD torques should oscillate around zero.
%exampleHelperLBRPlot(i-1, timePoints, feedForwardTorque, pdTorque, Q, QDesired )
