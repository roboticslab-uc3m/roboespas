clear all;
close all;
init_ros;
rosparam('set', '/iiwa_command/control_step_size', 0.01);

%%
%Move to a non-singularity position: IMPORTANT! If you dont do this, youll
%need to restart the whole system

[traj_comm, traj_out] = IiwaCommandStack.MoveJ(deg2rad([0 0 0 0 0 0 0]));
[traj_comm, traj_out] = IiwaCommandStack.MoveJ(deg2rad([-15 30 30 -60 0 90 30]));
result = IiwaCommandStack.ChangeVelAccJerk(1, 1, 1);

%% Test the velocity is correct
[traj_comm, traj_out] = IiwaCommandStack.MoveJ(deg2rad([0 0 0 0 0 0 0]));
%%
IiwaPlotter.joint_accelerations({traj_comm, traj_out}, ['b', 'r']); 
IiwaPlotter.joint_velocities({traj_comm, traj_out}, ['b', 'r']); 
IiwaPlotter.joint_positions({traj_comm, traj_out}, ['b', 'r']);
%% Return to normal
[traj_comm, traj_out] = IiwaCommandStack.MoveJ(deg2rad([-15 30 30 -60 0 90 30]));
result = IiwaCommandStack.ChangeVelAccJerk(0.5, 0.5, 0.5);
[traj_comm, traj_out] = IiwaCommandStack.MoveJ(deg2rad([0 0 0 0 0 0 0]));



