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
[jointTorquePub, jtMsg] = rospublisher('/iiwa_gazebo/joint_command');
jointStateSub = rossubscriber('/iiwa_gazebo/joint_state');
clockSub = rossubscriber('/clock'); %Subscribe to gazebo clock

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
%Fill mdl config msg
mdlConfig_msg.ModelName = 'iiwa';
mdlConfig_msg.UrdfParamName = 'robot_description';
mdlConfig_msg.JointNames = {'iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3',...
                  'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7'};
mdlConfig_msg.JointPositions = homeConfiguration(iiwa_robot);

%% Move the robot while plotting the feedback
%figure;
%hold on;
weights = [0.3, 0.8, 0.6, 0.6, 0.3, 0.2, 0.1];
Kp=100*weights;
Kd=2*weights;
i = 1;
jss_output=[];
traj_comm = IiwaTrajectory;
traj_comm.name='commanded';

call(mdlConfig_cli, mdlConfig_msg);
timeStart = receive(clockSub);
timeStart = timeStart.Clock_.seconds;
while i< size(traj_des.effort,1)
    %Check how much time has passed since the start
    timeCurrent = receive(clockSub);
    timeCurrent = timeCurrent.Clock_.seconds;
    timeFromStart = timeCurrent - timeStart;
    %Choose index of the point that should be sent next
    i = round(timeFromStart/control_step_size) +1;
    %Stop loop if i is out of the effort vector
    if i>=size(traj_des.effort,1)
        break;
    end
    %Read current joint state and save it in vector
    js = receive(jointStateSub);
    jss_output = [jss_output js];
    %Calculate PD compensation torque based on joint position and velocity
    %errors
    effortPD=Kp.*(traj_des.q(i,:)-js.Position') + Kd.*(traj_des.qdot(i,:)-js.Velocity');
    effort = traj_des.effort(i,:) + effortPD;
    %Send effort;
    jtMsg.Effort=effort;
    send(jointTorquePub, jtMsg);
    
    %Save commanded and read into vectors;
    jss_output = [jss_output js];
    traj_comm.effort = [traj_comm.effort; effort];
    traj_comm.q = [traj_comm.q; traj_des.q(i,:)];
    traj_comm.qdot = [traj_comm.qdot; traj_des.qdot(i,:)];
    traj_comm.qdotdot = [traj_comm.qdotdot; traj_des.qdotdot(i,:)];
    traj_comm.t = [traj_comm.t; traj_des.t(i,:)];
end
%Send point by point
% Transform into IiwaTrajectories
traj_output = IiwaTrajectory('output', jss_output);

%% Inspect Results

plotter.joint_efforts(traj_comm, traj_output);
% Plot and inspect the actual joint torques and positions versus the desired values. Note that with the feed-forward torque,
% the PD torques should oscillate around zero.
plotter.effortWithPD(traj_des, traj_comm);
plotter.joint_positions(traj_comm, traj_output);
