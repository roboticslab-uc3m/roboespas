clear all;
init_ros;
[MoveJASrv_cli, MoveJASrv_msg] = rosactionclient('MoveJ');
MoveJASrv_cli.ActivationFcn = @(~) disp('MoveJ action server active');
%figure; hold on;
%MoveJASrv_cli.FeedbackFcn = @(~,msg) IiwaPlotter.joint_position(msg.JointState.Position', msg.TimeFromStart);
MoveJASrv_cli.FeedbackFcn = @(~,msg) (1);
MoveJASrv_cli.ResultFcn = @(~,msg) disp('MoveJ action server result received');

%MoveJASrv_msg.JointPosition=[0.7 -1.2 0.7 -1 0.3 -0.5 0.5];
MoveJASrv_msg.JointPosition=[1 0 0 0 0 0 0];
resultMsg = sendGoalAndWait(MoveJASrv_cli, MoveJASrv_msg);

%%
traj_comm = IiwaTrajectory('commanded', resultMsg.TrajectoryCommanded);
traj_output = IiwaTrajectory('desired', resultMsg.TrajectoryJointState);

IiwaPlotter.joint_positions_compare(traj_comm, traj_output);