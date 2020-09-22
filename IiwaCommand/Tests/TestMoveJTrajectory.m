clear all;
close all;
init_ros;

plot_result=1;
qdot_perc = 0.4;

q_sub = rossubscriber('/iiwa_command/joint_state');
rosparam('set', '/iiwa_command/velocity', qdot_perc);
qdot_factor = rosparam('get', '/iiwa_command/velocity');
qdot_max = deg2rad(cell2mat(rosparam('get', '/iiwa/limits/joint_velocity')))';
qdot_max = qdot_factor*qdot_max;
control_step_size= rosparam('get', '/iiwa_command/control_step_size');
q_curr = q_sub.LatestMessage.Position;

%% Build trajectory given degrees moved
% Build goal position
deg_moved = 40;
%joint_moved = 6;
q_inc = ones(7,1)*deg2rad(deg_moved);
%q_inc(joint_moved) = deg2rad(deg_moved);
q_goal = q_curr+q_inc;

% Build trajectory given control_step_size and qdot_max
t_needed = q_inc./qdot_max;
t_total = max(t_needed);
t_total = ceil(t_total/control_step_size)*control_step_size;
t_array = 0:control_step_size:t_total;

n_points = size(t_array,2);
x = linspace(0,1,n_points);
q_array = q_curr + x.*q_inc;
q_array(:,end) = q_goal;

%Add flat area
q_array(:,end+1:end+ceil(n_points/3)) = ones(1, ceil(n_points/3)).*q_array(:,end);
t_array(:,end+1:end+ceil(n_points/3)) = t_array(end)+control_step_size:control_step_size:t_array(end)+t_array(end)/3+control_step_size;
%Mirror beginning
q_array(:,end+1:end+length(q_array)) = fliplr(q_array);
t_array(:,end+1:end+length(t_array)) = t_array(end)+control_step_size:control_step_size:t_array(end)*2+control_step_size;
n_points = size(t_array, 2);

%% Build action server and message to send
traj_theory = IiwaTrajectory('sent', n_points);
traj_theory.q = q_array';
traj_theory.t = t_array';

smoothing_max = 1e-10;
qdotdot_max = 5;
nSegments=100;
traj_spline = IiwaTrajectoryGeneration.BoundedSplineTrajectory(traj_theory, smoothing_max, nSegments);
while (max(max(traj_spline.qdotdot)) > qdotdot_max)
    smoothing_max = smoothing_max*10;
    traj_spline=IiwaTrajectoryGeneration.BoundedSplineTrajectory (traj_spline, smoothing_max, nSegments);
end
smoothing_max
%%
[traj_comm, traj_output] = IiwaCommand.MoveJTrajectory(traj_spline);

%%
close all
if (plot_result)    
    IiwaPlotter.joint_positions({traj_theory, traj_spline, traj_output}, ['m', 'b', 'r']);
    IiwaPlotter.joint_velocities({traj_spline, traj_output}, ['b', 'r']);
    IiwaPlotter.joint_accelerations({traj_spline, traj_output}, ['b', 'r']);
    IiwaPlotter.joint_position_error(traj_spline, traj_output, 'b');
end


%% Find delay between both signals

