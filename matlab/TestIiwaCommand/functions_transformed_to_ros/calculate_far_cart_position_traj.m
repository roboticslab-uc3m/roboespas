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
control_step_size=0.001;

% Selected parameters
nsamples=100;
total_time = 5;


%% Option 1: Linspace
traj_linspace=IiwaTrajectory('linspace');
traj_linspace.t = linspace(0, total_time, nsamples);
traj_linspace.x(:,1) = linspace(x_ini(1), x_goal(1), nsamples);
traj_linspace.x(:,2) = linspace(x_ini(2), x_goal(2), nsamples);
traj_linspace.x(:,3) = linspace(x_ini(3), x_goal(3), nsamples);
traj_linspace.x(:,4) = linspace(x_ini(4), x_goal(4), nsamples);
traj_linspace.x(:,5) = linspace(x_ini(5), x_goal(5), nsamples);
traj_linspace.x(:,6) = linspace(x_ini(6), x_goal(6), nsamples);


%% Option 2: Obtain velocity -> Obtain positions

traj_velocity = IiwaTrajectory('with velocity');

screwA2B = ScrewTheory.frameA2B(x_ini, x_goal);
%Divide the given angle into nsamples, as there are nsamples, there will be
%nsamples-1 jumps/movements from one angle to another
screw_inc = screwA2B / (nsamples-1);
t_inc = total_time / (nsamples-1);
traj_velocity.x(1,:)=x_ini;
traj_velocity.t(1,:)=0;
for i=2:nsamples
    traj_velocity.x(i,:) = ScrewTheory.transformframe(traj_velocity.x(i-1,:), screw_inc);
    traj_velocity.t(i,:) = traj_velocity.t(i-1,:) + t_inc;
end
IiwaPlotter.cartesian_positions({traj_linspace, traj_velocity}, ['b', 'g']);
IiwaPlotter.cartesian_frames({traj_velocity, traj_linspace}, ['b', 'g']);
IiwaPlotter.frame(x_ini, 'r' );
IiwaPlotter.frame(x_goal, 'r');
axis equal;
