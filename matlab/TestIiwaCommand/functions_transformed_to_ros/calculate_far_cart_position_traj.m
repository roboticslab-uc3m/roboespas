clear all
close all
q_ini=[0.7 -1.2 0.7 -1 0.3 -0.5 0.5];
q_goal=[-0.7 1.2 -0.7 1 -0.3 0.5 -0.5];
x_goal=ScrewTheory.ForwardKinematics(q_goal);
format long;

% Given parameters
velocity=1;
control_step_size=0.02;

% Selected parameters
total_time = 5;
nsamples = round(total_time/control_step_size)+1;
factor_correction = 0.1; %factor of multiplication of the velocity of correcction

%% Option 2(correct): Obtain velocity -> Obtain positions
%Find x, q, qdot offline
traj_straight = ScrewTheory.BuildStraightTrajectoryAtGivenVelocity(q_ini, x_goal, control_step_size, 1, 'commanded');

%% 
%Simulate control
cont = 1;
%id_curr=1;
q_curr = q_ini;
traj_output = IiwaTrajectory('output');

n=1;
x_ini=ScrewTheory.ForwardKinematics(q_ini);
qdot1=ScrewTheory.IDK_point(q_curr, traj_straight.xdot(1,:));
t_pauses=zeros(1, size(traj_straight.x,1));
timeStart = tic;
while (cont)
    time_from_start = toc(timeStart); %(n-1)*control_step_size;%
    id_curr =  round(time_from_start/control_step_size) +1; %n;%
    if (n~=1)
        %Simular que se mueve el robot -> No en Gazebo
        time_passed = time_from_start - traj_output.t(n-1);
        q_exp = traj_output.q(n-1,:)+ traj_output.qdot(n-1,:)*control_step_size; %time_passed
        q_curr=q_exp;
        %Simular error -> No en Gazebo
        %Add random error
%         m=fix(rand([1 7])+0.5);
%         m(~m)=-1;
%         error = rand(1,7).*m/150;
%         if (mod(time_from_start, 0.5) < control_step_size*2)
%             q_curr = q_curr + error;
%         end
    end
    if (id_curr>=(size(traj_straight.x,1)))
        traj_output.xdot(n,:) = [0 0 0 0 0 0];
        traj_output.qdot(n,:) = [0 0 0 0 0 0 0];
        traj_output.q(n,:) = q_curr;
        traj_output.t(n,:) = time_from_start;
        traj_output.x(n,:) = ScrewTheory.ForwardKinematics(traj_output.q(n,:));
        break;
    end  
    %Choose xdot/qdot to correct that error
    x_exp = traj_straight.x(id_curr,:);
    x_curr = ScrewTheory.ForwardKinematics(q_curr);
    x_next = traj_straight.x(id_curr+1,:);
    
    if (~isempty(traj_straight.xdot))
        xdot_exp_A = traj_straight.xdot(id_curr,:);
    else
        xdot_exp_A = ScrewTheory.screwA2B_A(x_exp, x_next)/control_step_size;
    end
    xdot_exp_S = ScrewTheory.tfscrew_A2S(xdot_exp_A, x_exp);
    xdot_err_A = ScrewTheory.screwA2B_A(x_curr, x_exp)/control_step_size;
    xdot_err_S = ScrewTheory.tfscrew_A2S(xdot_err_A, x_curr);
    
    xdot_S = xdot_exp_S + xdot_err_S*factor_correction;
    % Calculate joint velocity with Moore-Penrose pseudojacobian
    qdot = ScrewTheory.IDK_point(q_curr, xdot_S);
    %%
    %Fill traj_output vector
    traj_output.xdot(n,:) = xdot_exp_A;
    traj_output.qdot(n,:) = qdot;
    traj_output.q(n,:) = q_curr;
    traj_output.t(n,:) = time_from_start;
    traj_output.x(n,:) = ScrewTheory.ForwardKinematics(traj_output.q(n,:));
    %Update variables
    n=n+1;
    %Pause until next iteration
    t_pause = traj_straight.t(id_curr,:)- toc(timeStart)  + control_step_size;
    t_pauses(n,:) = t_pause;
    pause(t_pause);
end

%% Plot
IiwaPlotter.cartesian_positions({traj_straight, traj_output}, ['b', 'g']);
IiwaPlotter.cartesian_position_error(traj_straight, traj_output, 'b');
%IiwaPlotter.joint_positions({traj_straight, traj_output}, ['b', 'g']);
%IiwaPlotter.joint_position_error(traj_straight, traj_output, 'r');

IiwaPlotter.joint_velocities({traj_straight, traj_output}, ['b', 'g']);
%IiwaPlotter.cartesian_frames({traj_commanded, traj_output}, ['b', 'g'], 30);
%axis equal