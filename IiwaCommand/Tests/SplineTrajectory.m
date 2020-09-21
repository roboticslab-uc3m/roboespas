clear all;
close all;
%init_ros;
%IiwaCommand.MoveJ([0 0 0 0 0 0 0]);

qdot_factor = 0.8;
plot_feedback=0;
plot_result=1;
%q_sub = rossubscriber('/iiwa_command/joint_state');
qdot_max = IiwaRobot.ThDotmax';%deg2rad(cell2mat(rosparam('get', '/iiwa/limits/joint_velocity')))';
qdot_max = qdot_factor*qdot_max;
control_step_size= 0.005; %rosparam('get', '/iiwa_command/control_step_size');
q_curr = [0 0 0 0 0 0 0]'; %q_sub.LatestMessage.Position;

%% Build trajectory given degrees moved
% Build goal position
deg_moved = 10;
%joint_moved = 6;
q_inc = ones(7,1)*deg2rad(deg_moved);
%q_inc(joint_moved) = deg2rad(deg_moved);
q_goal = q_curr+q_inc;

% Build trajectory given control_step_size and qdot_max
t_needed = q_inc./qdot_max;
t_total = max(t_needed);
t_total = ceil(t_total/control_step_size)*control_step_size;
t_array = 0:control_step_size:t_total;

nPoints = size(t_array,2);
x = linspace(0,1,nPoints);
q_array = q_curr + x.*q_inc;
q_array(:,end) = q_goal;

%Mirror
q_array(:,end+1:end+nPoints) = fliplr(q_array);
t_array(:,end+1:end+nPoints) = t_array(end)+control_step_size:control_step_size:t_total*2+control_step_size;
nPoints = size(t_array, 2);

%% Build action server and message to send
traj_theory = IiwaTrajectory('theory', nPoints);
traj_theory.q = q_array';
traj_theory.t = t_array';

%% Build spline
traj_spline = IiwaTrajectoryGeneration.BoundedSplineTrajectory(traj_theory, 10e-12, control_step_size);

%%
close all
IiwaPlotter.joint_positions({traj_theory, traj_spline}, ['b', 'r']);
IiwaPlotter.joint_velocities({traj_theory, traj_spline}, ['b', 'r']);
IiwaPlotter.joint_accelerations({traj_theory, traj_spline}, ['b', 'r']);




