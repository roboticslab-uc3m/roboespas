clear all
% close all
plot_feedback=1;
plot_result=1;
init_ros
[MoveLASrv_cli, MoveLASrv_msg] = rosactionclient('MoveL');
MoveLASrv_cli.ActivationFcn = @(~) disp('MoveL action server active');
if (plot_feedback)
    figure; hold on;
    MoveLASrv_cli.FeedbackFcn = @(~,msg) IiwaPlotter.cartesian_position(IiwaScrewTheory.ForwardKinematics(msg.JointState.Position'), msg.TimeFromStart);
else
    MoveLASrv_cli.FeedbackFcn = @(~,msg) (1);
end
MoveLASrv_cli.ResultFcn = @(~,msg) disp('MoveL action server result received');
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
q_curr = IiwaCommand.ReadCurrentJointPosition();
MoveJASrv_msg.JointPosition=q_curr;
pause(0.2);
resultMsg = sendGoalAndWait(MoveJASrv_cli, MoveJASrv_msg);

%% MoveL

q_curr = IiwaCommand.ReadCurrentJointPosition();
x_curr = IiwaScrewTheory.ForwardKinematics(q_curr);
x_goal = x_curr;
x_goal(3) = x_goal(3)-0.15;
%q_goal = [-0.7 1.2 -0.7 1 -0.3 0.5 -0.5];
%q_goal = [0.7 -1.2 0.7 -1 0.3 -0.5 0.5];
%q_goal = [0 0 0 0 0 0 0];

MoveLASrv_msg.CartesianPosition=x_goal;
resultMsg = sendGoalAndWait(MoveLASrv_cli, MoveLASrv_msg);

%%
close all;
traj_theory = IiwaTrajectory('theory', resultMsg.XTheory, resultMsg.XdotTheory);
traj_comm = IiwaTrajectory('commanded', resultMsg.TrajectoryCommanded);
traj_output = IiwaTrajectory('output', resultMsg.TrajectoryRead);


if (plot_result)
    IiwaPlotter.joint_positions({traj_theory, traj_comm, traj_output}, ['g', 'b', 'r']);
    IiwaPlotter.cartesian_positions({traj_theory, traj_comm, traj_output}, ['g', 'b', 'r']);
    IiwaPlotter.joint_velocities({traj_theory, traj_comm, traj_output}, ['g', 'b', 'r']);
    IiwaPlotter.cartesian_position_error(traj_theory, {traj_output, traj_comm}, ['r', 'b']);
end