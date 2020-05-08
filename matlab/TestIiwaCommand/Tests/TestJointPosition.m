clear all;
init_ros;
[MoveJASrv_cli, MoveJASrv_msg] = rosactionclient('MoveJ');
MoveJASrv_cli.ActivationFcn = @(~) disp('MoveJ action server active');
MoveJASrv_cli.FeedbackFcn = @(~,msg) (1); %Clean feedback function so it doesn't print the result every time it receives something
MoveJASrv_cli.ResultFcn = @(~,msg) disp('MoveJ action server result received');

MoveJASrv_msg.JointPosition=[1.2 -1.2 0.6 -0.6 -0.7 0.7 0];
%MoveJASrv_msg.JointPosition=[0 0 0 0 0 0 0];
resultMsg = sendGoalAndWait(MoveJASrv_cli, MoveJASrv_msg);