clear all;
init_ros;
plot_result=1;

qdot_perc = 0.1;
deg_moved = 40;
nSegments = 100;
smoothing=1e-10;%qdotdot_factor=1;


q_sub = rossubscriber('/iiwa_command/joint_state');
rosparam('set', '/iiwa_command/velocity', qdot_perc);
qdot_factor = rosparam('get', '/iiwa_command/velocity');
qdot_max = deg2rad(cell2mat(rosparam('get', '/iiwa/limits/joint_velocity')))';
qdot_max = qdot_factor*qdot_max;
control_step_size= rosparam('get', '/iiwa_command/control_step_size');
qinc_max = (qdot_max*control_step_size)';
q_curr = q_sub.LatestMessage.Position;

%% Build trajectory given degrees moved
% Build goal position
q_inc = ones(7,1)*deg2rad(deg_moved);
q_goal = q_curr+q_inc;

% Build trajectory given control_step_size and qdot_max
t_needed = q_inc./(qdot_max*0.7);
t_total = max(t_needed);
t_total = ceil(t_total/control_step_size)*control_step_size;
t_array = 0:control_step_size:t_total;

n_points = size(t_array,2);
x = linspace(0,1,n_points);
q_array = q_curr + x.*q_inc;
q_array(:,end) = q_goal;
%% Build action server and message to send
traj_theory = IiwaTrajectory('theory', n_points);
traj_theory.q = q_array';
traj_theory.t = t_array';
traj_theory = IiwaTrajectoryGeneration.FillVelocityAndAcceleration(traj_theory);

% smoothing_max = 1e-10;
% qdotdot_max = qdotdot_factor*[0.6, 0.6, 0.6, 0.6, 1, 1, 1];
% traj_spline = IiwaTrajectoryGeneration.BoundedSplineTrajectory(traj_theory, smoothing_max, nSegments);
% factor = max(max(abs(traj_spline.qdotdot))./qdotdot_max);
% factor_prev = factor;
% while (factor>1 && factor_prev>=factor)
%     smoothing_max = smoothing_max*factor*factor;
%     traj_spline=IiwaTrajectoryGeneration.BoundedSplineTrajectory (traj_spline, smoothing_max, nSegments);
%     factor_prev = factor;
%     factor = max(max(abs(traj_spline.qdotdot))./qdotdot_max);
% end
%smoothing_max
%traj_spline=IiwaTrajectoryGeneration.BoundedSplineTrajectory(traj_theory, smoothing, nSegments);

%% Pause and mirror
%Add flat area
traj_spline.q(end+1:end+ceil(n_points/3),:) = ones(ceil(n_points/3),1).*traj_spline.q(end,:);
traj_spline.t(end+1:end+ceil(n_points/3),:) = traj_spline.t(end)+control_step_size:control_step_size:traj_spline.t(end)+traj_spline.t(end)/3+control_step_size;
%Mirror beginning
traj_spline.q(end+1:end+length(traj_spline.q),:) = flipud(traj_spline.q);
traj_spline.t(end+1:end+length(traj_spline.t),:) = traj_spline.t(end)+control_step_size:control_step_size:traj_spline.t(end)*2+control_step_size;
n_points = length(traj_spline.t);
traj_spline.npoints = n_points;
traj_spline = IiwaTrajectoryGeneration.FillVelocityAndAcceleration(traj_spline);

%%
[traj_comm, traj_output] = IiwaCommand.MoveJTrajectory(traj_spline);

%%
close all
if (plot_result)    
    IiwaPlotter.joint_positions({traj_theory, traj_spline, traj_comm, traj_output}, ['m', 'b', 'k', 'r']);
    IiwaPlotter.joint_positions({traj_spline, traj_comm, traj_output}, ['m', 'b', 'r']);
    IiwaPlotter.joint_velocities({traj_theory, traj_spline, traj_comm, traj_output}, ['m', 'b', 'k', 'r']);
    
    IiwaPlotter.joint_accelerations({traj_spline, traj_output}, ['b', 'r']);
    IiwaPlotter.joint_position_error(traj_spline, traj_output, 'b');
end


%% Find delay between both signals

