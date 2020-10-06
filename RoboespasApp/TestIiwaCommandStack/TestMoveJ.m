clear all;
close all;
init_ros;
rosparam('set', '/iiwa_command/control_step_size', 0.01);
%%
[traj_comm, traj_out] = IiwaCommandStack.MoveJ(deg2rad([0 0 0 0 0 0 0]));
[traj_comm, traj_out] = IiwaCommandStack.MoveJ(deg2rad([-15 30 30 -60 0 90 30]));
[traj_comm, traj_out] = IiwaCommandStack.MoveJ(deg2rad([0 0 0 0 0 0 0]));


%%
IiwaPlotter.joint_accelerations({traj_comm, traj_out}, ['b', 'r']); 
IiwaPlotter.joint_velocities({traj_comm, traj_out}, ['b', 'r']); 
IiwaPlotter.joint_positions({traj_comm, traj_out}, ['b', 'r']);