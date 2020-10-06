clear all;
init_ros;
q_sub = rossubscriber('/iiwa_command/joint_state');
qdot_max = deg2rad(cell2mat(rosparam('get', '/iiwa/limits/joint_velocity')))';
control_step_size= rosparam('get', '/iiwa_command/control_step_size');
qdot = deg2rad([80, 85, 100, 75, 130, 135, 135]);

%% Select acceleration
qdotdot = deg2rad([450, 300, 1000, 1000, 1000, 1000, 1000]);
%% Move to 0
qini = q_sub.LatestMessage.Position;
qend = [0 0 0 0 0 0 0];
traj_theory = IiwaTrajectoryGeneration.TrapezoidalVelocityProfileTrajectory(qini, qend, control_step_size, qdot, qdotdot, 'prueba');
[traj_comm, traj_output] = IiwaCommand.MoveJTrajectory(traj_theory);

%% Move to 90
pause(1);
qini = q_sub.LatestMessage.Position;
qend = qini;
qend(2) = deg2rad(45);
traj_theory = IiwaTrajectoryGeneration.TrapezoidalVelocityProfileTrajectory(qini, qend, control_step_size, qdot, qdotdot, 'prueba');
[traj_comm, traj_output] = IiwaCommand.MoveJTrajectory(traj_theory);

%% Plot
close all
IiwaPlotter.joint_positions({traj_theory, traj_comm, traj_output}, ['m', 'b', 'r']);
IiwaPlotter.joint_velocities({traj_theory, traj_comm, traj_output}, ['m', 'b', 'r']);
IiwaPlotter.joint_accelerations({traj_theory, traj_comm, traj_output}, ['m', 'b', 'r']);
IiwaPlotter.joint_position_error(traj_comm, traj_output, 'b');



