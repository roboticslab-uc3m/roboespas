clear all;
close all;
init_ros;
plot_result=1;

IiwaCommand.SetVelocityFactor(0.1);
[traj_comm, traj_output] = IiwaCommand.MoveJ2(deg2rad([0 0 0 0 0 0 0]));

%%
close all
if (plot_result)
    IiwaPlotter.joint_positions({traj_comm, traj_output}, ['b', 'r']);
    IiwaPlotter.cartesian_positions({traj_comm, traj_output}, ['b', 'r']);
    IiwaPlotter.joint_velocities({traj_comm, traj_output}, ['b', 'r']);
    IiwaPlotter.cartesian_position_error(traj_comm, traj_output, 'b');
end