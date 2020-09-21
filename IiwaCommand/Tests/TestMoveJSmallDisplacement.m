clear all;
close all;
init_ros;
plot_feedback=0;
plot_result=1;

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
q_goal = q_curr;
for i=1:7
    q_goal(i) = q_goal(i) -deg2rad(10);
end

%q_goal = [0, 0, 0, 0, 0, 0, 0];
MoveJASrv_msg.JointPosition=q_goal;
%MoveJASrv_msg.JointPosition=[-2.2725, 1.2797, -2.7501, 1.5968, -0.9615, -0.4337, -1.2612];
%MoveJASrv_msg.JointPosition=[0 0 0 0 0 0 0];
resultMsg = sendGoalAndWait(MoveJASrv_cli, MoveJASrv_msg);

%%
close all;
traj_comm = IiwaTrajectory('commanded', resultMsg.TrajectoryCommanded);
traj_output = IiwaTrajectory('output', resultMsg.TrajectoryRead);

if (plot_result)
    IiwaPlotter.joint_positions({traj_comm, traj_output}, ['b', 'r']);
    IiwaPlotter.time_stamps({traj_comm, traj_output}, ['b', 'r'])
    %IiwaPlotter.cartesian_positions({traj_comm, traj_output}, ['b', 'r']);
    %IiwaPlotter.joint_velocities({traj_comm, traj_output}, ['b', 'r']);
    %IiwaPlotter.cartesian_position_error(traj_comm, traj_output, 'b');
end