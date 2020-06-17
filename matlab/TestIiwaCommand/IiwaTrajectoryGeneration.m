classdef IiwaTrajectoryGeneration
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    methods (Static)
%% Build trajectory
        function traj = TrapezoidalVelocityTrajectory(qini, xgoal, ttotal, control_step_size, name)
            x_ini= ScrewTheory.ForwardKinematics(qini);
            xinc_A = ScrewTheory.screwA2B_A(x_ini, xgoal);

            axis_rot=xinc_A(4:6)/norm(xinc_A(4:6));
            angle=norm(xinc_A(4:6));
            
            axis_tras=xinc_A(1:3)/norm(xinc_A(1:3));
            dist=norm(xinc_A(1:3));
            
            
            [a_tras, tacc, tflat] = IiwaTrajectoryGeneration.GetTrapezoidalTrajectoryTimeParameters(ttotal, dist, IiwaRobot.CartAccMax, control_step_size);
            a_rot = IiwaTrajectoryGeneration.GetTrapezoidalTrajectoryAcceleration(angle, tacc, tflat);
            
            %Deparametrize
            traj = IiwaTrajectoryGeneration.DeparametrizeTrapezoidalVelocityTrajectory(name, x_ini, tacc, tflat, a_tras, axis_tras, a_rot, axis_rot, control_step_size);
      
            %Deparametrize
%             traj_para_pos = IiwaTrajectoryGeneration.TrapezoidalVelocityTrajectoryParameterized(tacc, tflat, a_tras, control_step_size, 'parametrized_pos');
%             traj_para_ori = IiwaTrajectoryGeneration.TrapezoidalVelocityTrajectoryParameterized(tacc, tflat, a_rot, control_step_size, 'parametrized_ori');
%            traj_2 = IiwaTrajectoryGeneration.Deparametrize(traj_para_pos, traj_para_ori, axis_tras, axis_rot, x_ini, name);
        end
        function traj = DeparametrizeTrapezoidalVelocityTrajectory(name, x_ini, tacc, tflat, a_tras, axis_tras, a_rot, axis_rot, step_size)
            if (isinf(tacc) || isinf(tflat))
                ME=MException('IiwaTrajectoryGeneration:infiniteTimes', 'Acceleration and flat times must be non infinite');
                throw(ME);
            end
            a_screw = [a_tras*axis_tras, a_rot*axis_rot];
            
            ttotal = tacc*2 +tflat;
            npoints = ttotal/step_size +1;
            
            traj = IiwaTrajectory(name, npoints);
            traj.t = (0:step_size:ttotal)';
            
            traj.xdotdot(1,:) = zeros(1, 6);
            traj.xdot(1,:) = zeros(1,6);
            traj.x(1,:) = x_ini;
            
            ids_acc = 2:round(tacc/step_size)+1;
            ids_flat = round(tacc/step_size)+1:round((tacc+tflat)/step_size)+1;
            ids_dec = round((tacc+tflat)/step_size)+1:round(ttotal/step_size)+1;
            
            for i=ids_acc
                traj.xdotdot(i,:) = a_screw;
                traj.xdot(i,:) = a_screw *traj.t(i);
                traj.x(i,:) = ScrewTheory.tfframe_A(x_ini, 0.5*a_screw*traj.t(i)*traj.t(i));
            end
            for i=ids_flat
                t = traj.t(i)-traj.t(ids_flat(1));
                traj.xdotdot(i,:) = zeros(1,6);
                traj.xdot(i,:) = traj.xdot(ids_flat(1), :);
                traj.x(i,:) = ScrewTheory.tfframe_A(traj.x(ids_flat(1),:), traj.xdot(ids_flat(1),:)*t);
            end
            for i=ids_dec
                t = traj.t(i) - traj.t(ids_dec(1));
                traj.xdotdot(i,:) = -a_screw;
                traj.xdot(i,:) = traj.xdot(ids_dec(1),:) - a_screw*t;
                traj.x(i,:) = ScrewTheory.tfframe_A(traj.x(ids_dec(1),:), traj.xdot(ids_dec(1),:)*t-0.5*a_screw*t*t);
            end
        end

        function a = GetTrapezoidalTrajectoryAcceleration(dtotal, tacc, tflat)
            a = dtotal/(tacc*tflat + tacc*tacc);
        end
        
        function [a, tacc, tflat] = GetTrapezoidalTrajectoryTimeParameters(ttotal, dtotal, a_max, time_step)
            %%First solve for maximum acceleration
            % Opt1:Using the roots function
            %p = [1/a_max, -ttotal, dtotal];
            %vflat_sols = roots(p);
            %Make sure ttotal is achievable with the given time_step
            ttotal = ceil(ttotal/time_step)*time_step; 
            a_min = (4*dtotal)/(ttotal*ttotal);
            %a_max=a_min;
            % Opt2: Using the formula for 2nd grade polynomials
            vflat_sols(1,:)=ttotal*a_max/2 + a_max*sqrt(ttotal*ttotal-4*dtotal/a_max)/2;
            vflat_sols(2,:)=ttotal*a_max/2 - a_max*sqrt(ttotal*ttotal-4*dtotal/a_max)/2;
            
            if (~isreal(vflat_sols))
                disp('Not enough time or acceleration to reach the position')
                a=inf;
                tacc=inf;
                tflat=0;
                return 
            end
            tflat_sols= ttotal -2*vflat_sols./a_max;
            tacc_sols = vflat_sols./a_max;
            %Select only those with t>0 and v>0
            vflat = vflat_sols(tflat_sols>0 & tacc_sols>0 & vflat_sols>0);
            tflat = tflat_sols(tflat_sols>0 & tacc_sols>0 & vflat_sols>0);
            tacc = tacc_sols(tflat_sols>0 & tacc_sols>0 & vflat_sols>0);
            %Fit the values to the discrete system
            %First find tacc multiple of control_step_size
            tacc = ceil(tacc/time_step)*time_step; %ceil to avoid incrementing a_flat
            %Get tflat for that tacc
            tflat = ttotal - 2*tacc;
            %Find acceleration for these values, which no longer will be
            %IiwaRobot.CartPosAccMax, to fit the trajectory correctly
            a = IiwaTrajectoryGeneration.GetTrapezoidalTrajectoryAcceleration(dtotal, tacc, tflat);
            vflat = tacc * a;
%             x1 = vflat*vflat/(2*a)
%             x2 = x1 + vflat*tflat
%             x3 = x2 + vflat*tacc -0.5*a*tacc*tacc
        end
        
        function traj_output = BuildStraightTrajectoryAtGivenVelocity(q_ini, x_goal, control_step_size, velocity, name)
            %velocity is a number from 0 to 1 expressing the percentage of
            %the maximum qdot used in the whole trajectory
            
            %Build a straight trajectory using a fixed total_time (high
            %enough so it doesnt get limited by qdot limit
            time_first_approach = 20;
            tic
            traj_straight = IiwaTrajectoryGeneration.TrapezoidalVelocityTrajectory(q_ini, x_goal, time_first_approach, control_step_size, 'straight');
            toc
            %Get joint positions and velocities needed to follow that
            %straight trajectory (in theory)
            traj_idk = ScrewTheory.FillJointPositionsFromCartesianPositions(traj_straight, q_ini);
            %For each joint, get the maximum velocity it reaches in the
            %whole trajectory
            max_qdot = max(abs(traj_idk.qdot));
            %Get the percentage of the maximum joint velocity
            percentage_qdot = max_qdot./IiwaRobot.ThDotmax;
            %Get the joint percentage that is nearer its joint velocity limits
            max_percentage = max(percentage_qdot);
            %Therefore, the time may be multiplied by this percentage to
            %find the minimum time needed to follow the trajectory without
            %reaching the limits. Divide it by 0.9 to leave a margin to
            %correct errors
            time_minimum = time_first_approach * (max_percentage/0.9);
            %Now apply the velocity factor to the time
            time_new = time_minimum/velocity;
            %And adjust it the control_step_size
            time_new = ceil(time_new/control_step_size)*control_step_size;
            %Finally recalculate the straight trajectory for this time
            traj_output = IiwaTrajectoryGeneration.TrapezoidalVelocityTrajectory(q_ini, x_goal, time_new, control_step_size, name);
        end
        
%                 
%         function traj = Deparametrize(traj_pos, traj_ori, axis_tras, axis_rot, xini, name)
%             traj = IiwaTrajectory(name);
%             traj.t = traj_pos.t;
%             traj.xdot = [traj_pos.xdot*axis_tras, traj_ori.xdot*axis_rot];
%             traj.xdotdot = [traj_pos.xdotdot*axis_tras, traj_ori.xdotdot*axis_rot];
%             for i=1:size(traj.t,1)
%                 traj.x(i,:) = ScrewTheory.tfframe_A(xini, [traj_pos.x(i)*axis_tras, traj_ori.x(i)*axis_rot]);
%             end
%         end
%         function traj = TrapezoidalVelocityTrajectoryParameterized(tacc, tflat, a, time_step, name)
%             %Returns a 1 coordinate trajectory with v0=0 and vend=0, given 
%             %a certain time for accelerating/deccelerating, a time in which
%             %the velocity maintains constant, the acceleration and the 
%             %time step, as well as the name for the given trajectory
%             traj=IiwaTrajectory(name);
%             if (isinf(tacc) || isinf(tflat))
%                 ME=MException('IiwaTrajectory:infiniteTimes', 'Acceleration and flat times must be non infinite');
%                 throw(ME);
%                 return;
%             end
%             ttotal = tacc*2 + tflat;
%             %Build parameterized trajectory
%             ids_acc = 2:round(tacc/time_step)+1;
%             ids_flat = round(tacc/time_step)+1:round((tacc+tflat)/time_step)+1;
%             ids_dec = round((tacc+tflat)/time_step)+1:round(ttotal/time_step)+1;
% 
%             traj.xdotdot(ids_acc,:) = a;
%             traj.xdotdot(ids_flat,:) = 0;
%             traj.xdotdot(ids_dec(1:end-1),:) = -a;
%             traj.xdotdot(ids_dec(end),:) = 0;
%             traj.t = (0:time_step:ttotal)';
%             traj.x = zeros(size(traj.t,1), 1);
%             traj.xdot = zeros(size(traj.t,1), 1);
%             traj.x(1) = 0;
%             traj.xdot(1) = 0;
%             for i=ids_acc(1:end)
%                 traj.x(i) = 0.5 * a * traj.t(i)*traj.t(i);
%                 traj.xdot(i) = a*traj.t(i);
%             end
%             for i=ids_flat(1:end)
%                t_=traj.t(i)-traj.t(ids_flat(1));
%                traj.x(i) = traj.x(ids_flat(1))+traj.xdot(ids_flat(1))*t_;
%                traj.xdot(i)=traj.xdot(ids_flat(1));
%             end
%             for i=ids_dec(1:end)
%                 t_=traj.t(i)-traj.t(ids_dec(1));
%                 traj.x(i) = traj.x(ids_dec(1)) + traj.xdot(ids_dec(1))*t_-0.5*a*t_*t_;
%                 traj.xdot(i) = traj.xdot(ids_dec(1)) -a*t_;
%             end
%         end

    end
end

