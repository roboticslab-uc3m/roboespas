%% Connect to ROS Network
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
load lbr_waypoints.mat
traj_des = IiwaTrajectory(lbr, 'desired', tWaypoints, qWaypoints, 0:0.1:5);

%load lbr_waypoints.mat
%tt_final = tWaypoints(end);
%tt = 0:control_step_size:tt_final;
%[tWaypoints_mod, qWaypoints_mod] = IiwaTrajectory.redireccionarInicioTrayectoria(tWaypoints, qWaypoints);
%traj_des = IiwaTrajectory(lbr, 'desired', tWaypoints_mod, qWaypoints_mod, tt);


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
sample_time=mean(traj_des.t(2:end)-traj_des.t(1:end-1));
while h < size(traj_des.effort,1)
    %Check how much time has passed since the start
    timeCurrent = receive(clockSub);
    timeCurrent = timeCurrent.Clock_.seconds;
    timeFromStart = timeCurrent - timeStart;
    %Choose index of the point that should be sent next
    h = ceil(timeFromStart/sample_time);
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
    jtMsg.Effort = effort;%traj_des.effort(h,:)
    send(jointTorquePub, jtMsg);
    
    %Save commanded and read into vectors;
    jss_output = [jss_output js];
    traj_comm.effort = [traj_comm.effort; effort];
    traj_comm.t = [traj_comm.t; timeFromStart];
end
%traj_withoutPD.name='withoutPD';
%Send point by point
% Transform into IiwaTrajectories
traj_output = IiwaTrajectory('output', jss_output);

%% Inspect Results
IiwaPlotter.joint_efforts_compare(traj_comm, traj_output);