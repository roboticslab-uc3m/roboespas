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
[jointTorque_pub, jt_msg] = rospublisher('/iiwa_gazebo/joint_command');
jointState_sub = rossubscriber('/iiwa_gazebo/joint_state');
[mdlConfig_cli, mdlConfig_msg] = rossvcclient('gazebo/set_model_configuration');

%% Create an LBR RigidBodyTree Object from URDF
lbr = importrobot('iiwa14.urdf');
lbr.DataFormat = 'row';
lbr.Gravity = [0 0 -9.80];
sample_time = 0.001; 
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
mdlConfig_msg.ModelName = 'mw_iiwa';
mdlConfig_msg.UrdfParamName = 'robot_description';
mdlConfig_msg.JointNames = {'mw_iiwa_joint_1', 'mw_iiwa_joint_2', 'mw_iiwa_joint_3',...
                  'mw_iiwa_joint_4', 'mw_iiwa_joint_5', 'mw_iiwa_joint_6', 'mw_iiwa_joint_7'};
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


%% Send from Matlab

% Specify PD gains.
weights = [0.3, 0.8, 0.6, 0.6, 0.3, 0.2, 0.1];
Kp = 100*weights;
Kd = 2* weights;

once = 1;

% Prepare for data logging.
feedForwardTorque = zeros(n, 7);
pdTorque = zeros(n, 7);
timePoints = zeros(n,1);
Q = zeros(n,7);
QDesired = zeros(n,7);
call(mdlConfigClient, mdlConfig_msg)
% Computed torque control is implemented in the for loop below. As soon as
% MATLAB receives a new joint state from Gazebo, it looks up in the 
% pre-generated tauFeedForward and finds the feed-forward torque 
% corresponding to the time stamp. It also computes a PD torque to 
% compensate for the errors in joint position and velocities [1].

% With default settings in Gazebo, the /iiwa_matlab_plugin/iiwa_matlab_joint_state
%  topic is updated at around 1 kHz (Gazebo sim time) with a typical 0.6 
% real time factor. And the torque control loop below can typically run 
% at around 200 Hz (Gazebo sim time).


clear torques_commanded
clear torques_read
for i = 1:size(points,2)
    % Get joint state from Gazebo.
    jsMsg = receive(jointState_sub);

    %Get timestamp
    t=jsMsg.Header.Stamp.seconds;
    % Set the start time.
    if i==1
        tStart = t;
    end
    % Find the corresponding index h in tauFeedForward vector for joint 
    % state time stamp t.
    tdiff=points(i+1).TimeFromStart.seconds-points(i).TimeFromStart.seconds;
    h = ceil((t - tStart + 1e-8)/tdiff)
    disp(['i: ', num2str(i)])
    if h>n
        disp('break');
    %    break
    end
    
    % Inquire feed-forward torque at the time when the joint state is
    % updated (Gazebo sim time).
    tau1 = points(h).Effort';%tauFeedForward(h,:);

    % Compute PD compensation torque based on joint position and velocity
    % errors.
    tau2 = Kp.*(points(h).Positions' - jsMsg.Position') + Kd.*(points(h).Velocities' - jsMsg.Velocity');
    tau = tau1 + tau2;

    % Send torque to Gazebo.
    jt_msg.Effort = tau;
    send(jointTorque_pub,jt_msg);    

    %Save info to plot afterwards
    feedForwardTorque(i,:) = tau1;
    torques_commanded(i,:)=tau;
    torques_read(i,:)=jsMsg.Effort';
    pdTorque(i,:) = tau2';
    timePoints(i) = t-tStart;
    Q(i,:) = jsMsg.Position';
    QDesired(i,:) = qDesired(h,:);  
end

% With the joint torques sent, the LBR robot should follow the trajectory. This image shows snapshots of the robot overlaid
% throughout the trajectory.

%% Compare torques
figure;
for i=1:7
    subplot(7,1,i);
    plot(torques_commanded(:,i));
    hold on;
    plot(torques_read(:,i));
    legend('commanded', 'read');
end
%% Inspect Results

% Plot and inspect the actual joint torques and positions versus the desired values. Note that with the feed-forward torque,
% the PD torques should oscillate around zero.
exampleHelperLBRPlot(i-1, timePoints, feedForwardTorque, pdTorque, Q, QDesired )
