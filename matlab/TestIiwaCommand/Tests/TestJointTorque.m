%% Connect to ROS Network
clear all;
close all;
% Connect with ROS master
init_ros;
ptree=rosparam;
% Create actionlib to send a trajectory to gazebo iiwa robot, and some
% services to create and read the joint_trajectory messages faster
[jointTorquePub, jtMsg] = rospublisher('/iiwa_gazebo/joint_command');
jointStateSub = rossubscriber('/iiwa_gazebo/joint_state');
clockSub = rossubscriber('/clock'); %Subscribe to gazebo clock

[mdlConfig_cli, mdlConfig_msg] = rossvcclient('gazebo/set_model_configuration');
plotter=IiwaPlotter();
%% Create an LBR RigidBodyTree Object from URDF
control_step_size = get(ptree, '/iiwa_command/control_step_size');
lbr = importrobot('iiwa14.urdf');
lbr.DataFormat = 'row';
lbr.Gravity = [0 0 -9.80];

%% Load trajectory
%trayectoria = 2;
%[tWaypoints, qWaypoints] = IiwaTrajectory.ExampleTrajectoryIJL(trayectoria);
load lbr_waypoints.mat
%   qWaypoints =    [0 0 0 0 0 0 0;
%                    qWaypoints(1,:)
%                     qWaypoints(end,:)];
% 
%   tWaypoints = [0,1,3];

tt_final = tWaypoints(end);
tt = 0:control_step_size:tt_final;
[tWaypoints_mod, qWaypoints_mod] = IiwaTrajectory.redireccionarInicioTrayectoria(tWaypoints, qWaypoints);
traj_des = IiwaTrajectory(lbr, 'desired', tWaypoints_mod, qWaypoints_mod, tt);

%% Present first and last joint positions before adding home position
%First position
lbr.Bodies{1,2}.Joint.HomePosition = qWaypoints(1,1);
lbr.Bodies{1,3}.Joint.HomePosition = qWaypoints(1,2);
lbr.Bodies{1,4}.Joint.HomePosition = qWaypoints(1,3);
lbr.Bodies{1,5}.Joint.HomePosition = qWaypoints(1,4);
lbr.Bodies{1,6}.Joint.HomePosition = qWaypoints(1,5);
lbr.Bodies{1,7}.Joint.HomePosition = qWaypoints(1,6);
lbr.Bodies{1,8}.Joint.HomePosition = qWaypoints(1,7);
figure; hold on;
show(lbr);
%Last position
lbr.Bodies{1,2}.Joint.HomePosition = qWaypoints(end,1);
lbr.Bodies{1,3}.Joint.HomePosition = qWaypoints(end,2);
lbr.Bodies{1,4}.Joint.HomePosition = qWaypoints(end,3);
lbr.Bodies{1,5}.Joint.HomePosition = qWaypoints(end,4);
lbr.Bodies{1,6}.Joint.HomePosition = qWaypoints(end,5);
lbr.Bodies{1,7}.Joint.HomePosition = qWaypoints(end,6);
lbr.Bodies{1,8}.Joint.HomePosition = qWaypoints(end,7);
show(lbr);

view([150 12]);
axis([-0.8 0.8 -0.8 0.8 0 1.35]);
camva(9);
daspect([1 1 1]);

%Correct home position
lbr.Bodies{1,2}.Joint.HomePosition = qWaypoints_mod(1,1);
lbr.Bodies{1,3}.Joint.HomePosition = qWaypoints_mod(1,2);
lbr.Bodies{1,4}.Joint.HomePosition = qWaypoints_mod(1,3);
lbr.Bodies{1,5}.Joint.HomePosition = qWaypoints_mod(1,4);
lbr.Bodies{1,6}.Joint.HomePosition = qWaypoints_mod(1,5);
lbr.Bodies{1,7}.Joint.HomePosition = qWaypoints_mod(1,6);
lbr.Bodies{1,8}.Joint.HomePosition = qWaypoints_mod(1,7);

%% Reset LBR to Home Configuration in Gazebo
%Fill mdl config msg
mdlConfig_msg.ModelName = 'iiwa';
mdlConfig_msg.UrdfParamName = 'robot_description';
mdlConfig_msg.JointNames = {'iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3',...
                  'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7'};
mdlConfig_msg.JointPositions = homeConfiguration(lbr);

%% Move the robot while plotting the feedback
%figure;
%hold on;
weights = [0.3, 0.8, 0.6, 0.6, 0.3, 0.2, 0.1];
Kp=100*weights;
Kd=2*weights;
h = 1;
jss_output=[];
traj_comm = IiwaTrajectory;
traj_comm.name ='commanded';

call(mdlConfig_cli, mdlConfig_msg);
timeStart = receive(clockSub);
timeStart = timeStart.Clock_.seconds;
while h < size(traj_des.effort,1)
    %Check how much time has passed since the start
    timeCurrent = receive(clockSub);
    timeCurrent = timeCurrent.Clock_.seconds;
    timeFromStart = timeCurrent - timeStart;
    %Choose index of the point that should be sent next
    h = round(timeFromStart/control_step_size) +1;
    %Stop loop if i is out of the effort vector
    if h>=size(traj_des.effort,1)
        break;
    end
    %Read current joint state and save it in vector
    js = receive(jointStateSub);
    %Calculate PD compensation torque based on joint position and velocity
    %errors
    effortPD = Kp.*(traj_des.q(h,:)-js.Position') + Kd.*(traj_des.qdot(h,:)-js.Velocity');
    effort = traj_des.effort(h,:) + effortPD;
    %Send effort;
    jtMsg.Effort = effort;
    send(jointTorquePub, jtMsg);
    
    %Save commanded and read into vectors;
    jss_output = [jss_output js];
    traj_comm.effort = [traj_comm.effort; effort];
    traj_comm.q = [traj_comm.q; traj_des.q(h,:)];
    traj_comm.qdot = [traj_comm.qdot; traj_des.qdot(h,:)];
    traj_comm.qdotdot = [traj_comm.qdotdot; traj_des.qdotdot(h,:)];
    traj_comm.t = [traj_comm.t; traj_des.t(h,:)];
    
    traj_withoutPD = IiwaTrajectory(traj_comm);
    traj_withoutPD.effort(end,:) = traj_des.effort(h,:);
end
traj_withoutPD.name='withoutPD';
%Send point by point
% Transform into IiwaTrajectories
traj_output = IiwaTrajectory('output', jss_output);

%% Inspect Results
IiwaPlotter.joint_efforts_compare(traj_comm, traj_output);
% Plot and inspect the actual joint torques and positions versus the desired values. Note that with the feed-forward torque,
% the PD torques should oscillate around zero.
IiwaPlotter.effortWithPD(traj_des, traj_comm);
%IiwaPlotter.effortWithPD_compare_big(traj_comm, traj_output, traj_withoutPD);

%Compare joint positions
IiwaPlotter.joint_positions_compare(traj_comm, traj_output);
%IiwaPlotter.joint_positions_compare_big(traj_comm, traj_output);
IiwaPlotter.joint_effort_error(traj_comm, traj_output);

IiwaPlotter.joint_position_error(traj_comm, traj_output);


%% Save Results
% t_measured = traj_output.t;
% Q = traj_output.q;
% Qdot = traj_output.qdot;
% tau_ID = traj_withoutPD.effort;
% tau_real = traj_output.effort;
% 
% save('Input-Dataset-Trayectory9.mat','t_measured','Q','Qdot','tau_ID','tau_real')
