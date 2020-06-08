clear all
close all
init_ros
[MoveLASrv_cli, MoveLASrv_msg] = rosactionclient('MoveL');
q_goal = [-0.7 1.2 -0.7 1 -0.3 0.5 -0.5];
x_goal = ScrewTheory.ForwardKinematics(q_goal);
MoveLASrv_msg.CartesianPosition=x_goal;
sendGoalAndWait(MoveLASrv_cli, MoveLASrv_msg);