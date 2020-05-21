clear all
close all
q_ini=[0.7 -1.2 0.7 -1 0.3 -0.5 0.5];
q_goal=[-0.7 1.2 -0.7 1 -0.3 0.5 -0.5];
x_goal=ScrewTheory.ForwardKinematics(q_goal);

% Given parameters
qdotmax=[75 75 90 90 144 135 135];
qdotmax_rad=deg2rad(qdotmax);
max_inc_rad = 0.003;
velocity=1;
control_step_size=0.001;

% Selected parameters
total_time = 5;
nsamples = total_time/control_step_size+1;
factor_correction = 0.01; %Time it takes to correct 1

%% Option 2(correct): Obtain velocity -> Obtain positions
%Find x positions
traj_commanded = IiwaTrajectory('commanded');

x_ini=ScrewTheory.ForwardKinematics(q_ini);
xinc_total_A = ScrewTheory.screwA2B_A(x_ini, x_goal);
xinc_A = xinc_total_A / (nsamples-1);
t_inc = total_time / (nsamples-1);
traj_commanded.x(1,:)=x_ini;
traj_commanded.q(1,:)=q_ini;
traj_commanded.t(1,:)=0;
for i=2:nsamples
    q_curr=traj_commanded.q(i-1,:);
    x_curr=traj_commanded.x(i-1,:);
    
    %x_next = ScrewTheory.tfframe_A(x_curr, xinc_A);
    %xinc_A = ScrewTheory.screwA2B_A (x_curr, x_next);   
    xdot_A = xinc_A/control_step_size;
    traj_commanded.xdot(i-1,:) = xdot_A;
    
    xdot_S = ScrewTheory.tfscrew_A2S(xdot_A, x_curr);
    qdot = ScrewTheory.IDK_point(q_curr, xdot_S);
    traj_commanded.qdot(i-1,:) = qdot;
    
    q_next = q_curr + qdot*control_step_size;
    traj_commanded.q(i,:) = q_next;
    
    x_next = ScrewTheory.ForwardKinematics(q_next);
    traj_commanded.x(i,:) = x_next;
    
    t_next = traj_commanded.t(i-1,:) + t_inc;
    traj_commanded.t(i,:) = t_next;
end
traj_commanded.qdot(nsamples,:)=zeros(1,7);
traj_commanded.xdot(nsamples,:)=zeros(1,6);

%Find q positions for that x positions
cont = 1;
%id_curr=1;
q_curr = q_ini;
traj_output = IiwaTrajectory('output');

n=1;
timeStart = tic;
while (cont)
    time_from_start = toc(timeStart);
    id_curr = round(time_from_start/control_step_size) +1;
    if (id_curr>(size(traj_commanded.x,1)))
        break;
    end
    
    if (n~=1)
        time_passed=time_from_start - traj_output.t(n-1);
        q_curr = traj_output.q(n-1,:) + traj_output.qdot(n-1,:)*time_passed;
        
        %Add random error
        m=fix(rand([1 7])+0.5);
        m(~m)=-1;
        error = rand(1,7).*m/100;
        if (mod(time_from_start, 0.5) < control_step_size*2)
            q_curr = q_curr + error;
            disp(num2str(n));
            disp(num2str(time_from_start));
        end
    end
    
    %Choose precalculated qdot/xdot
    xdot = traj_commanded.xdot(id_curr,:);
    qdot = traj_commanded.qdot(id_curr,:);
    
    %Choose xdot/qdot to correct that error
    x_curr = ScrewTheory.ForwardKinematics(q_curr);
    q_exp = traj_commanded.q(id_curr,:);
    x_exp = ScrewTheory.ForwardKinematics(q_exp);
    xdot_expected_A = traj_commanded.xdot(id_curr, :);
    xdot_correct_A = (ScrewTheory.screwA2B_A(x_curr, x_exp))/control_step_size;
    xdot_A = xdot_expected_A + xdot_correct_A*0.01;
    xdot_S = ScrewTheory.tfscrew_A2S(xdot_A, x_curr);
    qdot = ScrewTheory.IDK_point(q_curr, xdot_S);
    xdot = xdot_A;
    
    
    %Fill traj_output vector
    traj_output.xdot(n,:) = xdot;
    traj_output.qdot(n,:) = qdot;
    traj_output.q(n,:) = q_curr;
    traj_output.t(n,:) = time_from_start;
    traj_output.x(n,:) = ScrewTheory.ForwardKinematics(traj_output.q(n,:));
    
    %Update variables
    q_curr = q_next;
    n=n+1;
    %Pause until next iteration
    pause(traj_commanded.t(id_curr,:) + control_step_size - toc(timeStart));
end

%% Plot
IiwaPlotter.cartesian_positions({traj_commanded, traj_output}, ['b', 'g']);
IiwaPlotter.joint_positions({traj_commanded, traj_output}, ['b', 'g']);
IiwaPlotter.joint_position_error(traj_commanded, traj_output, 'r');

IiwaPlotter.joint_velocities({traj_commanded, traj_output}, ['b', 'g']);
%IiwaPlotter.cartesian_frames({traj_commanded, traj_output}, ['b', 'g'], 30);
%axis equal