classdef IiwaTrajectoryGeneration
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        segments_per_deg_spline = 1;
    end
    methods (Static, Access = 'public')
        function traj_output = FillVelocityAndAcceleration(traj_input)
            traj_output = traj_input;
            css = mean(traj_input.t(2:end)-traj_input.t(1:end-1));
            traj_output.t = (0:css:css*(traj_input.npoints-1))';
            qdot_med = (traj_input.q(2:end,:)-traj_input.q(1:end-1,:))./css; %Mean velocities at css/2, css/2+css, css/2 + css*2, ...
            qdot = (qdot_med(1:end-1,:)+qdot_med(2:end,:))./2;
            traj_output.qdot = [zeros(1, IiwaRobot.n_joints); qdot; zeros(1, IiwaRobot.n_joints)];
            qdotdot_med = (traj_output.qdot(2:end,:)-traj_output.qdot(1:end-1,:))./css; %Mean velocities at css/2, css/2+css, css/2 + css*2, ...
            qdotdot = (qdotdot_med(1:end-1,:)+qdotdot_med(2:end,:))./2;
            traj_output.qdotdot = [zeros(1, IiwaRobot.n_joints); qdotdot; zeros(1, IiwaRobot.n_joints)];
        end
        function traj_output = TrapezoidalVelocityProfileTrajectory(q_ini, q_goal, control_step_size, qdot, qdotdot, name)
            for i=1:7
                if (qdot(i)*control_step_size>abs(q_goal(i)-q_ini(i)))
                    [~, ~, ~, tSamples, ~] = trapveltraj([q_ini(i), q_goal(i)], 100, 'PeakVelocity', qdot(i));
                else
                    [~, ~, ~, tSamples, ~] = trapveltraj([q_ini(i), q_goal(i)], 100, 'PeakVelocity', qdot(i), 'Acceleration', qdotdot(i));
                end
                times(i)=tSamples(end);
            end
            ttot = max(times);
            npoints = ceil(ttot/control_step_size);
            for i=1:7
                if (qdot(i)*control_step_size>abs(q_goal(i)-q_ini(i)))
                    [q(i,:), qd(i,:), qdd(i,:), t(i,:), pp] = trapveltraj([q_ini(i), q_goal(i)], npoints, 'PeakVelocity', qdot(i));
                else
                    [q(i,:), qd(i,:), qdd(i,:), t(i,:), pp] = trapveltraj([q_ini(i), q_goal(i)], npoints, 'PeakVelocity', qdot(i), 'Acceleration', qdotdot(i));
                end
            end
            traj_output = IiwaTrajectory(name,npoints);
            traj_output.q=q';
            traj_output.qdot=qd';
            traj_output.qdotdot=qdd';
            traj_output.t=t(1,:)';
        end
%         function traj_output = TrapezoidalVelocityProfileTrajectory(q_ini, x_goal, control_step_size, velocity, name)
%             %velocity is a number from 0 to 1 expressing the percentage of
%             %the maximum qdot used in the whole trajectory
%             
%             %Build a straight trajectory using a fixed total_time (high
%             %enough so it doesnt get limited by qdot limit
%             time_first_approach = 20;
%             tic
%             traj_straight = IiwaTrajectoryGeneration.TrapezoidalVelocityTrajectory(q_ini, x_goal, time_first_approach, control_step_size, 'straight');
%             toc
%             %Get joint positions and velocities needed to follow that
%             %straight trajectory (in theory)
%             traj_idk = IiwaScrewTheory.FillJointPositionsFromCartesianPositions(traj_straight, q_ini);
%             %For each joint, get the maximum velocity it reaches in the
%             %whole trajectory
%             max_qdot = max(abs(traj_idk.qdot));
%             %Get the percentage of the maximum joint velocity
%             percentage_qdot = max_qdot./IiwaRobot.ThDotmax;
%             %Get the joint percentage that is nearer its joint velocity limits
%             max_percentage = max(percentage_qdot);
%             %Therefore, the time may be multiplied by this percentage to
%             %find the minimum time needed to follow the trajectory without
%             %reaching the limits. Divide it by 0.9 to leave a margin to
%             %correct errors
%             time_minimum = time_first_approach * (max_percentage/0.9);
%             %Now apply the velocity factor to the time
%             time_new = time_minimum/velocity;
%             %And adjust it the control_step_size
%             time_new = ceil(time_new/control_step_size)*control_step_size;
%             %Finally recalculate the straight trajectory for this time
%             traj_fastest = IiwaTrajectoryGeneration.TrapezoidalVelocityTrajectory(q_ini, x_goal, time_new, control_step_size, name);
%             traj_output = IiwaScrewTheory.FillJointPositionsFromCartesianPositions(traj_fastest, q_ini);
%         end
% %% Build Trapezoidal Trajectory
%         function traj = TrapezoidalVelocityTrajectory(qini, xgoal, ttotal, control_step_size, name)
%             x_ini= IiwaScrewTheory.ForwardKinematics(qini);
%             xinc_A = IiwaScrewTheory.screwA2B_A(x_ini, xgoal);
% 
%             axis_rot=xinc_A(4:6)/norm(xinc_A(4:6));
%             angle=norm(xinc_A(4:6));
%             
%             axis_tras=xinc_A(1:3)/norm(xinc_A(1:3));
%             dist=norm(xinc_A(1:3));
%             
%             
%             [a_tras, tacc, tflat] = IiwaTrajectoryGeneration.GetTrapezoidalTrajectoryTimeParameters(ttotal, dist, IiwaRobot.CartAccMax, control_step_size);
%             a_rot = IiwaTrajectoryGeneration.GetTrapezoidalTrajectoryAcceleration(angle, tacc, tflat);
%             
%             %Deparametrize
%             traj = IiwaTrajectoryGeneration.DeparametrizeTrapezoidalVelocityTrajectory(name, x_ini, tacc, tflat, a_tras, axis_tras, a_rot, axis_rot, control_step_size);
%         end
%         function traj = DeparametrizeTrapezoidalVelocityTrajectory(name, x_ini, tacc, tflat, a_tras, axis_tras, a_rot, axis_rot, step_size)
%             if (isinf(tacc) || isinf(tflat))
%                 ME=MException('IiwaTrajectoryGeneration:infiniteTimes', 'Acceleration and flat times must be non infinite');
%                 throw(ME);
%             end
%             a_screw = [a_tras*axis_tras, a_rot*axis_rot];
%             
%             ttotal = tacc*2 +tflat;
%             npoints = ttotal/step_size +1;
%             
%             traj = IiwaTrajectory(name, npoints);
%             traj.t = (0:step_size:ttotal)';
%             
%             traj.xdotdot(1,:) = zeros(1, 6);
%             traj.xdot(1,:) = zeros(1,6);
%             traj.x(1,:) = x_ini;
%             
%             ids_acc = 2:round(tacc/step_size)+1;
%             ids_flat = round(tacc/step_size)+1:round((tacc+tflat)/step_size)+1;
%             ids_dec = round((tacc+tflat)/step_size)+1:round(ttotal/step_size)+1;
%             
%             for i=ids_acc
%                 traj.xdotdot(i,:) = a_screw;
%                 traj.xdot(i,:) = a_screw *traj.t(i);
%                 traj.x(i,:) = IiwaScrewTheory.tfframe_A(x_ini, 0.5*a_screw*traj.t(i)*traj.t(i));
%             end
%             for i=ids_flat
%                 t = traj.t(i)-traj.t(ids_flat(1));
%                 traj.xdotdot(i,:) = zeros(1,6);
%                 traj.xdot(i,:) = traj.xdot(ids_flat(1), :);
%                 traj.x(i,:) = IiwaScrewTheory.tfframe_A(traj.x(ids_flat(1),:), traj.xdot(ids_flat(1),:)*t);
%             end
%             for i=ids_dec
%                 t = traj.t(i) - traj.t(ids_dec(1));
%                 traj.xdotdot(i,:) = -a_screw;
%                 traj.xdot(i,:) = traj.xdot(ids_dec(1),:) - a_screw*t;
%                 traj.x(i,:) = IiwaScrewTheory.tfframe_A(traj.x(ids_dec(1),:), traj.xdot(ids_dec(1),:)*t-0.5*a_screw*t*t);
%             end
%         end
%         function a = GetTrapezoidalTrajectoryAcceleration(dtotal, tacc, tflat)
%             a = dtotal/(tacc*tflat + tacc*tacc);
%         end       
%         function [a, tacc, tflat] = GetTrapezoidalTrajectoryTimeParameters(ttotal, dtotal, a_max, time_step)
%             %%First solve for maximum acceleration
%             % Opt1:Using the roots function
%             %p = [1/a_max, -ttotal, dtotal];
%             %vflat_sols = roots(p);
%             %Make sure ttotal is achievable with the given time_step
%             ttotal = ceil(ttotal/time_step)*time_step; 
%             a_min = (4*dtotal)/(ttotal*ttotal);
%             %a_max=a_min;
%             % Opt2: Using the formula for 2nd grade polynomials
%             vflat_sols(1,:)=ttotal*a_max/2 + a_max*sqrt(ttotal*ttotal-4*dtotal/a_max)/2;
%             vflat_sols(2,:)=ttotal*a_max/2 - a_max*sqrt(ttotal*ttotal-4*dtotal/a_max)/2;
%             
%             if (~isreal(vflat_sols))
%                 disp('Not enough time or acceleration to reach the position')
%                 a=inf;
%                 tacc=inf;
%                 tflat=0;
%                 return 
%             end
%             tflat_sols= ttotal -2*vflat_sols./a_max;
%             tacc_sols = vflat_sols./a_max;
%             %Select only those with t>0 and v>0
%             vflat = vflat_sols(tflat_sols>0 & tacc_sols>0 & vflat_sols>0);
%             tflat = tflat_sols(tflat_sols>0 & tacc_sols>0 & vflat_sols>0);
%             tacc = tacc_sols(tflat_sols>0 & tacc_sols>0 & vflat_sols>0);
%             %Fit the values to the discrete system
%             %First find tacc multiple of control_step_size
%             tacc = ceil(tacc/time_step)*time_step; %ceil to avoid incrementing a_flat
%             %Get tflat for that tacc
%             tflat = ttotal - 2*tacc;
%             %Find acceleration for these values, which no longer will be
%             %IiwaRobot.CartPosAccMax, to fit the trajectory correctly
%             a = IiwaTrajectoryGeneration.GetTrapezoidalTrajectoryAcceleration(dtotal, tacc, tflat);
%             vflat = tacc * a;
% %             x1 = vflat*vflat/(2*a)
% %             x2 = x1 + vflat*tflat
% %             x3 = x2 + vflat*tacc -0.5*a*tacc*tacc
%         end
%% Build bounded spline trajectory
        function traj_spline = BoundedSplineTrajectory (traj_in, smoothing, nSegments)
            if (isempty(traj_in.t))
                disp('IiwaTrajectoryGeneration:BoundedSplineTrajectory:Empty time vector');
                return 
            end
            t = traj_in.t';
            q = traj_in.q';
            q_inc_max = max(traj_in.q) - min(traj_in.q);
            %nSegments = ceil(IiwaTrajectoryGeneration.segments_per_deg_spline*rad2deg(max(abs(q_inc_max))));
            nKnots=nSegments+1;
            tKnot=linspace(t(1), t(end), nKnots);    

            %% Run main function
            [H, f] = IiwaTrajectoryGeneration.computeQuadraticCostMatrix(t', q', tKnot', smoothing);
            nState = size(q,1);
            [A1, b1] = IiwaTrajectoryGeneration.computeEqCstContinuity(tKnot, nState);
            lowBnd.pos = q(:,1)';
            lowBnd.vel = zeros(1, size(q,1));
            lowBnd.acc = [];%zeros(1, size(q,1));
            uppBnd.pos = q(:,end)';
            uppBnd.vel = zeros(1, size(q,1));
            uppBnd.acc = [];%zeros(1, size(q,1));
            [A2, b2] = IiwaTrajectoryGeneration.computeEqCstBoundary(lowBnd, uppBnd, tKnot);
            coeff = IiwaTrajectoryGeneration.solveQpEq(H,f,[A1;A2],[b1;b2]);
            nPoly = 4;
            pp.form = 'pp';
            pp.breaks = tKnot;
            pp.coefs = zeros(nSegments*nState, nPoly);  % 4 = cubic coeff count
            pp.pieces = nSegments;
            pp.order = nPoly;
            pp.dim = nState;
            for iSeg = 1:nSegments
                for iDim = 1:nState
                  idx_pp_row = nState*(iSeg-1) + iDim;
                  idx_coef_cols = nPoly*(iSeg-1) + (1:nPoly);
                  pp.coefs(idx_pp_row,:) = fliplr(coeff(idx_coef_cols,iDim)');
                end
            end
            qpp = mkpp(pp.breaks, pp.coefs, pp.dim);
            qdotpp = IiwaTrajectoryGeneration.ppDer(qpp);
            qdotdotpp = IiwaTrajectoryGeneration.ppDer(qdotpp);
            % Fill output
            traj_spline = IiwaTrajectory('spline', length(t));
            traj_spline.t=t';
            traj_spline.q=(ppval(qpp, t))';
            traj_spline.qdot=(ppval(qdotpp, t))';
            traj_spline.qdotdot=(ppval(qdotdotpp, t))';
        end
        function dpp = ppDer(pp)
            % Computes the time-derivative of piece-wise polynomial (PP) struct
            % INPUTS:
            %   pp = a PP struct containing a trajectory of interest
            % OUTPUTS:
            %   dpp = a new PP struct that is the time-derivative of pp
            % NOTES:
            %   --> a pp struct is typically created by matlab functions such as
            %   spline, pchip, or pwch and evaluated using ppval.
            %     pp=mkpp(breaks,coefs);
            %     n = pp.order;
            %     nRows = size(pp.coefs,1);
            %     coefs = zeros(nRows,n-1);
            %     for i=1:n-1
            %        coefs(:,i) = (n-i)*pp.coefs(:,i);
            %     end
            %     dpp=mkpp(pp.breaks, coefs);
            n = pp.order;
            nRows = size(pp.coefs,1);
            dpp.form = pp.form;
            dpp.breaks = pp.breaks;
            dpp.coefs = zeros(nRows,n-1);
            for i=1:n-1
               dpp.coefs(:,i) = (n-i)*pp.coefs(:,i);
            end
            dpp.pieces = pp.pieces;
            dpp.order = pp.order-1;
            dpp.dim = pp.dim;
        end
        function [H, f] = computeQuadraticCostMatrix(tData, xData, tKnot, smooth)
            % This function computes the least-squares cost terms for fitting a
            % cubic spline to time-series data. Includes a term for minimizing the
            % integral of jerk-squared as well.
            % INPUTS:
            %   tData = [nTime, 1] = time-series data
            %   xData = [nTime, nState] = state data at each point
            %   tKnot = [nKnot, 1] = knot points for cubic spline
            %   smooth = weight on jerk-squared
            % OUTPUTS:
            %   H = [4*(nKnot-1), 4*(nKnot-1)] = quadratic cost matrix
            %   f = [4*(nKnot-1), nState] = linear cost matrix
            % NOTES:
            %   nSeg = nKnot - 1;
            %   z = [4*nKnot, nState];
            %   z = [C0; C1; ... CN] coefficient vector
            %   CI = [aI; bi; ci; di];  x = a + b*t +c*t^2 + d*t^3
            %   J = 0.5*z'*H*z + f'*z;  % cost function
            % Figure out which data points belong to which segments
            nKnot = length(tKnot);
            nSeg = nKnot - 1;
            [~, binIdx] = histc(tData, tKnot);
            binIdx(binIdx == nKnot) = nKnot;  % include points at endpoint in last segment
            nState = size(xData,2);
            % Accumulate cost matrix:
            beta = (tData(end) - tData(1))/length(tData);
            H = zeros(nSeg*4, nSeg*4);
            f = zeros(nSeg*4, nState);
            for iBin = 1:nSeg
               tBin = tData(binIdx==iBin);
               xBin = xData(binIdx==iBin, :);
               tZero = tKnot(iBin);
               idx = 4*(iBin-1) + (1:4);
               for iData = 1:length(tBin)
                   [hTmp, fTmp] = IiwaTrajectoryGeneration.singleDataPoint(tBin(iData) - tZero, xBin(iData,:));
                 H(idx, idx) = H(idx, idx) +  hTmp;
                 f(idx,:) = f(idx,:) + fTmp;
               end
               hSeg = tKnot(iBin+1) - tKnot(iBin);
               H(idx, idx) = beta*H(idx, idx) + smooth*IiwaTrajectoryGeneration.minJerk(hSeg);
            end
            f = beta*f;
        end
        function [H, f] = singleDataPoint(t, x)
            % Computes the quadratic cost associated with the fitting error between a
            % single point and a cubic segment.
            t0 = 1;
            t1 = t;
            t2 = t1*t1;
            t3 = t2*t1;
            t4 = t2*t2;
            t5 = t2*t3;
            t6 = t3*t3;
            H = [t0, t1, t2, t3; t1, t2, t3, t4; t2, t3, t4, t5; t3, t4, t5, t6;];
            f = [-x*t0; -x*t1; -x*t2; -x*t3;];
        end
        function H = minJerk(h)
            H = zeros(4,4);
            H(4,4) = 36*h;
        end
        function [A, b] = computeEqCstBoundary(low, upp, tKnot)
            % This function computes the equality constraints that enforce the
            % boundary position, velocity, and acceleration.
            % INPUTS:
            %   low.pos = [1, nState] = position at lower boundary
            %   low.vel = [1, nState] = velocity at lower boundary
            %   low.acc = [1, nState] = acceleration at lower boundary
            %   upp.pos = [1, nState] = position at upper boundary
            %   upp.vel = [1, nState] = velocity at upper boundary
            %   upp.acc = [1, nState] = acceleration at upper boundary
            %   nKnot = [scalar] = number of knot points on the spline
            % OUTPUTS:
            %   A = [3*nSeg, 4*nSeg] = linear constraint matrix
            %   b = zeros(3*nSeg, nState) = constant terms in constraint
            % NOTES:
            %   Set any boundary constraint to [] to ignore it.
            %
            %   nSeg = nKnot - 1;
            %   z = [4*nKnot, nState];
            %   z = [C0; C1; ... CN] coefficient vector
            %   CI = [aI; bi; ci; di];  x = a + b*t +c*t^2 + d*t^3
            %   A*z = b;  % cost function
            b = [ low.pos; low.vel; low.acc; upp.pos; upp.vel; upp.acc;];
            nGrid = length(tKnot);
            nDecVar = 4*(nGrid-1);
            if ~isempty(low.pos)
                aLowPos = [1,0,0,0, zeros(1, nDecVar-4)];
            else
                aLowPos = [];
            end
            if ~isempty(low.vel)
                aLowVel = [0,1,0,0, zeros(1, nDecVar-4)];
            else
                aLowVel = [];
            end
            if ~isempty(low.acc)
                aLowAcc = [0,0,2,0, zeros(1, nDecVar-4)];
            else
                aLowAcc = [];
            end
            % Upper edge of the last segment
            h1 = tKnot(end) - tKnot(end-1); 
            h2 = h1*h1;
            h3 = h2*h1;
            if ~isempty(upp.pos)
                aUppPos = [zeros(1, nDecVar-4), 1,   h1,    h2,      h3];
            else
                aUppPos = [];
            end
            if ~isempty(upp.vel)
                aUppVel = [zeros(1, nDecVar-4), 0,   1,   2*h1,    3*h2];
            else
                aUppVel = [];
            end
            if ~isempty(upp.acc)
                aUppAcc = [zeros(1, nDecVar-4), 0,   0,     2,     6*h1];
            else
                aUppAcc = [];
            end
            A = [aLowPos; aLowVel; aLowAcc; aUppPos; aUppVel; aUppAcc;];
        end
        function [A, b] = computeEqCstContinuity(tKnot, nState)
            % This function computes the equality constraints that enforce position,
            % velocity, and acceleration continuity.
            % INPUTS:
            %   tKnot = [nKnot, 1] = knot points for cubic spline
            % OUTPUTS:
            %   A = [3*nSeg, 4*nSeg] = linear constraint matrix
            %   b = zeros(3*nSeg, nState) = constant terms in constraint
            % NOTES:
            %   nSeg = nKnot - 1;
            %   z = [4*nKnot, nState];
            %   z = [C0; C1; ... CN] coefficient vector
            %   CI = [aI; bi; ci; di];  x = a + b*t +c*t^2 + d*t^3
            %   A*z = b;  % cost function
            nKnot = length(tKnot);
            nSeg = nKnot - 1;
            b = zeros(3*(nSeg-1), nState);
            A = zeros(3*(nSeg-1), 4*nSeg);
            for iSeg = 1:(nSeg-1)
              rowIdx = 3*(iSeg-1) + (1:3);
              colIdx = 4*(iSeg-1) + (1:4);
              hSeg = tKnot(iSeg+1) - tKnot(iSeg);
              [upp, low] = IiwaTrajectoryGeneration.continuityConstraint(hSeg);
              A(rowIdx, colIdx) = A(rowIdx, colIdx) + upp;
              A(rowIdx, colIdx+4) = A(rowIdx, colIdx+4) - low;
            end
        end
        function [upp, low] = continuityConstraint(h)
            h2 = h*h;
            h3 = h2*h;
            % upper edge of the lower segment:
            upp = [ 1,   h,    h2,    h3;
                    0,   1,  2*h,   3*h2;
                    0,   0,    2,   6*h];
            % lower edge of the upper segment:
            low = [ 1,   0,  0,  0;
                    0,   1,  0,  0;
                    0,   0,  2,  0];
        end
        function [zSoln, lambda] = solveQpEq(H,f,A,b)
            % min 0.5*x'*H*x + f'*x   subject to:  A*x = b
            % Linear solve formulation:
            % [H, Aeq'; Aeq, 0] * [z;w] = [-f;beq]
            % MM*xx = cc
            s = norm(H);
            H = H/s;
            f = f/s;
            s = norm(A);
            A = A/s;
            b = b/s;
            nCst = size(A,1);
            nDecVar = size(H,1);
            MM = [H, A'; A, zeros(nCst)];
            cc = [-f; b];
            xx = MM \ cc;
            zSoln = xx(1:nDecVar,:);
            lambda = xx((nDecVar+1):end,:);
        end
    end
end

