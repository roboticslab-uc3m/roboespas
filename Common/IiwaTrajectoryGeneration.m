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
            sub_q = traj_input.q(2:end,:)-traj_input.q(1:end-1, :);
            sub_q = medfilt1(sub_q, round(traj_input.npoints/30));
            traj_output.t = (0:css:css*(traj_input.npoints-1))';
            qdot_med = sub_q./css; %Mean velocities at css/2, css/2+css, css/2 + css*2, ...
            qdot = (qdot_med(1:end-1,:)+qdot_med(2:end,:))./2;
            traj_output.qdot = [zeros(1, IiwaRobot.n_joints); qdot; zeros(1, IiwaRobot.n_joints)];
            sub_qdot = traj_output.qdot(2:end,:)-traj_output.qdot(1:end-1,:);
            sub_qdot = medfilt1(sub_qdot, round(traj_input.npoints/30));
            qdotdot_med = sub_qdot./css; %Mean velocities at css/2, css/2+css, css/2 + css*2, ...
            qdotdot = (qdotdot_med(1:end-1,:)+qdotdot_med(2:end,:))./2;
            traj_output.qdotdot = [zeros(1, IiwaRobot.n_joints); qdotdot; zeros(1, IiwaRobot.n_joints)];
        end
        function traj_output = FillCartesian(traj_input)
            traj_output = traj_input;
            for i=1:size(traj_output.q,1)
                traj_output.x(i,:)=IiwaScrewTheory.ForwardKinematics(traj_output.q(i,:));
            end
        end
        function traj_output = TrapezoidalVelocityProfileTrajectory(q_ini, q_goal, control_step_size, qdot, qdotdot, name)
            times=zeros(1,7);
            for i=1:7
                nsamples = round(abs(q_goal(i)-q_ini(i))/(qdot(i)*control_step_size));
                if (qdot(i)*control_step_size < abs(q_goal(i)-q_ini(i)))
                    [~, ~, ~, tSamples, ~] = trapveltraj([q_ini(i), q_goal(i)], nsamples, 'PeakVelocity', qdot(i), 'Acceleration', qdotdot(i));
                else
                    traj_output=IiwaTrajectory(0, name);
                    continue;
                end
%                 if (qdot(i)*control_step_size/2>abs(q_goal(i)-q_ini(i)))
%                     [~, ~, ~, tSamples, ~] = trapveltraj([q_ini(i), q_goal(i)], 100, 'PeakVelocity', qdot(i));
%                 else
%                     [~, ~, ~, tSamples, ~] = trapveltraj([q_ini(i), q_goal(i)], 100, 'Acceleration', qdotdot(i));
%                 end
                times(i)=tSamples(end);
            end
            ttot = max(times);
            npoints = ceil(ttot/control_step_size);
            for i=1:7
                if (qdot(i)*control_step_size<abs(q_goal(i)-q_ini(i)))
                    [q(i,:), qd(i,:), qdd(i,:), t(i,:), pp] = trapveltraj([q_ini(i), q_goal(i)], npoints, 'PeakVelocity', qdot(i), 'Acceleration', qdotdot(i));
                else
                    if (q_ini(i)==q_goal(i))
                        q(i,:) = ones(1, npoints)*q_ini(i);
                        qd(i,:) = zeros(1,npoints);
                        qdd(i,:) = zeros(1,npoints);
                    else
                        [q(i,:), qd(i,:), qdd(i,:), t(i,:), pp] = trapveltraj([q_ini(i), q_goal(i)], npoints, 'PeakVelocity', abs(q_ini(i)-q_goal(i))/(control_step_size/2));
                    end
                end
            end
            traj_output = IiwaTrajectory(name,npoints);
            traj_output.q=q';
            traj_output.qdot=qd';
            traj_output.qdotdot=qdd';
            traj_output.t=t(1,:)';
        end
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
            traj_spline = IiwaTrajectoryGeneration.FillCartesian(traj_spline);
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

