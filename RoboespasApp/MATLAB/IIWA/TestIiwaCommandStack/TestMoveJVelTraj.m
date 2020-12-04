clear all;
close all;
init_ros;
rosparam('set', '/iiwa_command/control_step_size', 0.5);
%% Move the robot to a joint position to record a trajectory to repeat afterwards
[~, ~] = IiwaCommandStack.MoveJ(deg2rad([0 0 0 0 0 0 0]));
[~, ~] = IiwaCommandStack.MoveJ(deg2rad([-15 30 30 -60 0 90 30]));
IiwaCommandStack.ChangeVelAccJerk(0.1, 0.5, 0.5);
[~, ~] = IiwaCommandStack.MoveJ(deg2rad([0 0 0 0 0 0 0]));
[~, traj_des] = IiwaCommandStack.MoveJ(deg2rad([-15 30 30 -60 0 90 30]));
traj_spline = IiwaTrajectoryGeneration.BoundedSplineTrajectory(traj_des, 10e-4, 100);
%%
IiwaCommandStack.ChangeVelAccJerk(1,1,1);
[traj_comm, traj_out] = IiwaCommandStack.MoveJVelTraj(traj_spline);
IiwaCommandStack.ChangeVelAccJerk(0.5, 0.5, 0.5);
%%
close all;
IiwaPlotter.joint_accelerations({traj_comm, traj_out}, ['b', 'r']); 
IiwaPlotter.joint_velocities({traj_comm, traj_out}, ['b', 'r']); 
IiwaPlotter.joint_positions({traj_comm, traj_out}, ['b', 'r']);


