clear all
close all
q_ini=[0.7 -1.2 0.7 -1 0.3 -0.5 0.5];
q_goal=[-0.7 1.2 -0.7 1 -0.3 0.5 -0.5];
x_goal=ScrewTheory.ForwardKinematics(q_goal);
x_ini=ScrewTheory.ForwardKinematics(q_ini);

% Given parameters
qdotmax=[75 75 90 90 144 135 135];
qdotmax_rad=deg2rad(qdotmax);
max_inc_rad = 0.003;
velocity=1;
control_step_size=0.01;

% Selected parameters
nsamples=1000;
total_time = 5;
%% Calculate cartesian limits from joint limits


%% Option 1 (wrong): Linspace
% traj_linspace=IiwaTrajectory('linspace');
% traj_linspace.t = linspace(0, total_time, nsamples);
% traj_linspace.x(:,1) = linspace(x_ini(1), x_goal(1), nsamples);
% traj_linspace.x(:,2) = linspace(x_ini(2), x_goal(2), nsamples);
% traj_linspace.x(:,3) = linspace(x_ini(3), x_goal(3), nsamples);
% traj_linspace.x(:,4) = linspace(x_ini(4), x_goal(4), nsamples);
% traj_linspace.x(:,5) = linspace(x_ini(5), x_goal(5), nsamples);
% traj_linspace.x(:,6) = linspace(x_ini(6), x_goal(6), nsamples);


%% Option 2(correct): Obtain velocity -> Obtain positions

traj_commanded = IiwaTrajectory('commanded');
xinc_A = ScrewTheory.frameA2B_A(x_ini, x_goal);
x_inc = xinc_A / (nsamples-1);
t_inc = total_time / (nsamples-1);
traj_commanded.x(1,:)=x_ini;
traj_commanded.t(1,:)=0;
for i=2:nsamples
    traj_commanded.x(i,:) = ScrewTheory.transformframe_A(traj_commanded.x(i-1,:), x_inc);
    traj_commanded.t(i,:) = traj_commanded.t(i-1,:) + t_inc;
end

cont = 1;
n=1;
q_curr = q_ini;
traj_followed = IiwaTrajectory('output');
t_curr = 0;
t_sample_mean = mean(traj_commanded.t(2:end)-traj_commanded.t(1:end-1));
time = t_sample_mean;
timeStart = tic;
while (cont)
    time_from_start = toc(timeStart);
    h = round(time_from_start/t_sample_mean) +1;
    if (h>=(size(traj_commanded.x,1)))
        break;
    end
    x_next = traj_commanded.x(h+1, :);
    x_curr = ScrewTheory.ForwardKinematics(q_curr);
    xinc_A = ScrewTheory.frameA2B_A (x_curr, x_next);
    xinc_S = ScrewTheory.transformscrewA2S(xinc_A, x_curr);
    xdot_S = xinc_S/time;
    qdot = ScrewTheory.IDK_point(q_curr, xdot_S);
    q_next = q_curr + qdot*control_step_size;
    traj_followed.q(n,:) = q_curr;
    traj_followed.t(n,:) = time_from_start;
    traj_followed.x(n,:) = ScrewTheory.ForwardKinematics(q_curr);
    q_curr = q_next;
    n=n+1;
end
% for i = 1: size(traj_commanded.x,1)-1
%     x_next = traj_commanded.x(i+1,:);
%     % Calculate current cartesian position using forward kinematics
%     x_curr=ScrewTheory.ForwardKinematics(q_curr);
%     %
%     xinc_A = ScrewTheory.frameA2B_A(x_curr, x_next);
%     % Transform velocity in screw form
%     xinc_S = ScrewTheory.transformscrewA2S(xinc_A, x_curr);
%     xdot_S = xinc_S/time;
%     qdot = ScrewTheory.IDK_point(q_curr, xdot_S);
% 
%     q_next = q_curr + qdot*t_sample_mean;
%     traj_followed.q(i,:) = q_curr;
%     traj_followed.t(i,:) = t_curr;
%     traj_followed.x(i,:) = ScrewTheory.ForwardKinematics(q_curr);
%     
%     t_curr = t_curr + t_sample_mean;
%     q_curr = q_next;
% end

%% Plot
IiwaPlotter.cartesian_positions({traj_commanded, traj_followed}, ['b', 'g']);
%IiwaPlotter.cartesian_frames({traj_commanded, traj_followed}, ['b', 'g']);
%axis equal