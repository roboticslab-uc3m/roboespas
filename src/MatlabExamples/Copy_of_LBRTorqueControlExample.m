%% Connect to ROS Network from MATLAB&reg;

% In your MATLAB instance on the host computer, run the following commands 
% to initialize ROS global node in MATLAB and connect
% to the ROS master in the virtual machine (where Gazebo is running) through
% its IP address. Replace ipaddress with the IP address of your virtual machine.

if (robotics.ros.internal.Global.isNodeActive)
    rosshutdown;
end
setenv('ROS_MASTER_URI', 'http://192.168.1.53:11311');
setenv('ROS_IP', '192.168.1.53');
rosinit;

% Establish Communication Channel With Gazebo Through Customized Topics

% Gazebo provides two ROS services /gazebo/get_joint_properties and 
% /gazebo/apply_joint_effort that can be used to get joint state and set 
% joint torques. However, the services are too slow to close the torque control
% loop. Therefore, a customized Gazebo plug-in is used so that the joint 
% state/torques in Gazebo can be read/written at a much faster rate through
% the plain ROS topics (publisher and subscriber). The customized Gazebo 
% plug-in is already brought up together with Gazebo LBR Simulator.

[jointTorquePub, jtMsg] = rospublisher('/iiwa_gazebo/joint_command');
jointStateSub = rossubscriber('/iiwa_gazebo/joint_state');

%% Create an LBR RigidBodyTree Object from URDF
lbr = importrobot('iiwa14.urdf');
lbr.DataFormat = 'row';

% Set the gravity to be the same as that in Gazebo.
lbr.Gravity = [0 0 -9.80];

% Show home configuration in a MATLAB figure.
show(lbr);
view([150 12]);
axis([-0.6 0.6 -0.6 0.6 0 1.35]);
camva(9);
daspect([1 1 1]);

% Pre-Compute Joint Torque Trajectory for Desired Motion

% Load joint configuration waypoints. This gives the key frames for the 
% desired motion of the robot.
load lbr_waypoints.mat

% sample_time is the planned control stepsize. We use it to populate a set of time 
% points where the trajectory needs to be evaluated and store it in vector
% tt.
sample_time = 0.001; 
tt = 0:sample_time:5;

% Generate desired motion trajectory for each joint.
% exampleHelperJointTrajectoryGeneration generates joint trajectories from 
% given time and joint configuration waypoints. 
% The trajectories are generated using pchip so that the interpolated 
% joint position does not violate joint limits as long as the waypoints do not.
[qDesired, qdotDesired, qddotDesired, tt] = exampleHelperJointTrajectoryGeneration(tWaypoints, qWaypoints, tt);

% Pre-compute feed-forward torques that ideally would realize the desired motion (assuming no disturbances or any kind of errors)
% using inverseDynamics. The following for loop takes some time to run. 
% To accelerate, consider used generated code for inverseDynamics. 
% See the last section for details on how to do it.
n = size(qDesired,1);
tauFeedForward = zeros(n,7);
for i = 1:n
    tauFeedForward(i,:) = inverseDynamics(lbr, qDesired(i,:), qdotDesired(i,:), qddotDesired(i,:));
end

%% Reset LBR to Home Configuration in Gazebo
% Initialize empty vector
tic
clear points;
for i=1:size(tauFeedForward,1)
    points(i)=rosmessage('trajectory_msgs/JointTrajectoryPoint');
end
for i=1:size(tauFeedForward,1)
    points(i).Effort=tauFeedForward(i,:);
    points(i).Positions=qDesired(i,:);
    points(i).Velocities=qdotDesired(i,:);
    points(i).Accelerations=qddotDesired(i,:);
    points(i).TimeFromStart=rosduration(tt(i));
end
toc
%Fill trajectory
[client, goalMsg] = rosactionclient('iiwa_command');
waitForServer(client);
goalMsg.TrajectoryDesired.JointNames= {'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'};
goalMsg.TrajectoryDesired.Points=points;
%client.FeedbackFcn = @(~) disp('a');
sendGoal(client, goalMsg);
%while (client.GoalState=='active')
%    disp(['Feedback: ',showdetails(msg)]);
%end
%%

% Given: tauFeedForward, qDesired, qdotDesired, cdt
% Use Gazebo-provided service to reset the robot to its home configuration. 
% For details on how to work with ROS service in MATLAB, see Call and
% Provide ROS Services. 
mdlConfigClient = rossvcclient('gazebo/set_model_configuration');

% Compose the required service message. It includes the joint names and 
% corresponding joint positions to send to Gazebo. Call the service using 
% this message.
msg = rosmessage(mdlConfigClient);
msg.ModelName = 'mw_iiwa';
msg.UrdfParamName = 'robot_description';
msg.JointNames = {'mw_iiwa_joint_1', 'mw_iiwa_joint_2', 'mw_iiwa_joint_3',...
                  'mw_iiwa_joint_4', 'mw_iiwa_joint_5', 'mw_iiwa_joint_6', 'mw_iiwa_joint_7'};
msg.JointPositions = homeConfiguration(lbr);

call(mdlConfigClient, msg);

% Computed Torque Control

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
call(mdlConfigClient, msg)
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
for i = 1:n
    % Get joint state from Gazebo.
    jsMsg = receive(jointStateSub);

    %Get timestamp
    t=jsMsg.Header.Stamp.seconds;
    % Set the start time.
    if once
        tStart = t;
        once = 0;
    end
    % Find the corresponding index h in tauFeedForward vector for joint 
    % state time stamp t.
    h = ceil((t - tStart + 1e-8)/sample_time);
    if h>n
        break
    end
    
    % Inquire feed-forward torque at the time when the joint state is
    % updated (Gazebo sim time).
    tau1 = tauFeedForward(h,:);
    % Log feed-forward torque.
    feedForwardTorque(i,:) = tau1;
    
    % Compute PD compensation torque based on joint position and velocity
    % errors.
    q = jsMsg.Position';
    qdot=jsMsg.Velocity';
    tau2 = Kp.*(qDesired(h,:) - q) + Kd.*(qdotDesired(h,:) - qdot);
    tau = tau1 + tau2;

    % Send torque to Gazebo.
    jtMsg.Effort = tau;
    send(jointTorquePub,jtMsg);    

    %Save info to plot afterwards
    torques_commanded(i,:)=tau;
    torques_read(i,:)=jsMsg.Effort';
    pdTorque(i,:) = tau2';
    timePoints(i) = t-tStart;
    Q(i,:) = q';
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
