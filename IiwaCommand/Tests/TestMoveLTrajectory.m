%% Move the robot following a certain trajectory
clear all;
close all;
init_ros;
plot_feedback = 1;
plot_result = 1;
load('/home/leytha/roboespas_ws/src/roboespas/matlab/TestIiwaCommand/functions_transformed_to_ros/prueba.mat')
x= [Trial.Trajectory.CartesianTrajectory.Position; Trial.Trajectory.CartesianTrajectory.Orientation];
t = Trial.Trajectory.Timestamps;
q_ini = Trial.Trajectory.JointTrajectory(:,1);
twist_stamped_vec = IiwaMsgTransformer.toTwistStampedVec(x,t);
%Move to first position 
[MoveJASrv_cli, MoveJASrv_msg] = rosactionclient('MoveJ');
MoveJASrv_cli.ActivationFcn = @(~) disp('MoveJ action server active');
if (plot_feedback)
    figure; hold on;
    MoveJASrv_cli.FeedbackFcn = @(~,msg) IiwaPlotter.joint_position(msg.JointState.Position', msg.TimeFromStart);
else
    MoveJASrv_cli.FeedbackFcn = @(~,msg) (1);
end
MoveJASrv_cli.ResultFcn = @(~,msg) disp('MoveJ action server result received');
MoveJASrv_msg.JointPosition=q_ini';
pause(0.2);
resultMsg = sendGoalAndWait(MoveJASrv_cli, MoveJASrv_msg);

%Follow the trajectory
[MoveLTrajASrv_cli, MoveLTrajASrv_msg] = rosactionclient('MoveLTrajectory');
MoveLTrajASrv_cli.ActivationFcn = @(~) disp('MoveLTraj action server active');
if (plot_feedback)
    figure; hold on;
    MoveLTrajASrv_cli.FeedbackFcn = @(~,msg) IiwaPlotter.joint_position(msg.JointState.Position', msg.TimeFromStart);
else
    MoveLTrajASrv_cli.FeedbackFcn = @(~,msg) (1);
end
MoveLTrajASrv_cli.ResultFcn = @(~,msg) disp('MoveLTraj action server result received');
MoveLTrajASrv_msg.TrajectoryGoal = twist_stamped_vec;
MoveLTrajASrv_msg.QIni = q_ini;

resultMsg = sendGoalAndWait(MoveLTrajASrv_cli, MoveLTrajASrv_msg);
close all;

%%
traj_comm = IiwaTrajectory('commanded', resultMsg.TrajectoryCommanded);
traj_output = IiwaTrajectory('output', resultMsg.TrajectoryRead);

if (plot_result)
    IiwaPlotter.joint_positions({traj_comm, traj_output}, ['b', 'r']);
    IiwaPlotter.cartesian_positions({traj_comm, traj_output}, ['b', 'r']);
    IiwaPlotter.joint_velocities({traj_comm, traj_output}, ['b', 'r']);
    IiwaPlotter.cartesian_position_error(traj_comm, traj_output, 'b');
    IiwaPlotter.joint_position_error(traj_comm, traj_output, 'b');
end
