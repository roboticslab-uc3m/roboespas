classdef IiwaTrajectory
    %IIWAINTERPOLATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access=public)
        t
        q
        qdot
        qdotdot
        effort
        x
        xdot
        xdotdot
        name
        npoints
    end
    properties (Constant)
        MinCartVelocity = 0.02 %m/s
        PointsForCircle = 20;
    end 
    methods
        function obj = IiwaTrajectory(varargin) %lbr, name, tWaypoints, qWaypoints, t)
            if (isempty(varargin))
                obj.npoints=0;
                return;
            end
            if (length(varargin)>=1)
                if (isa(varargin{1},'char'))
                    obj.name=varargin{1};
                end
            end
            if (length(varargin)>=2)
                if(isa(varargin{2},'robotics.ros.msggen.trajectory_msgs.JointTrajectory') || ...
                    isa(varargin{2},'robotics.ros.msggen.sensor_msgs.JointState'))
                    obj=IiwaMsgTransformer.toIiwaTrajectory(obj.name, varargin{2});
                    obj=obj.CompleteVelAcc();
                    obj=obj.CompleteCartesian();
                    return;
                end 
            end
            if (length(varargin)==3)
                %First parameter is the name, second is x, and third is
                %xdot
                obj=IiwaMsgTransformer.toIiwaTrajectory(varargin{1}, varargin{2}, varargin{3});
            elseif (length(varargin)==2)
                if (isa (varargin{2}, 'double'))
                    %First parameter is the name, and second parameter is npoints
                    obj.name = varargin{1};
                    obj.npoints = varargin{2};
                    obj.t = zeros(obj.npoints, 1);
                    obj.q = zeros(obj.npoints, 7);
                    obj.qdot = zeros(obj.npoints,7);
                    obj.qdotdot = zeros(obj.npoints,7);
                    obj.x = zeros(obj.npoints, 6);
                    obj.xdot = zeros(obj.npoints,6);
                    obj.xdotdot = zeros(obj.npoints,6);
                else
                    %First parameter is name and second is the ros msg
                    obj=IiwaMsgTransformer.toIiwaTrajectory(varargin{1}, varargin{2});
                end
            elseif (length(varargin)==1)
                if (ischar(varargin{1}))
                    obj.name=varargin{1};
                elseif (isa(varargin{1}, 'IiwaTrajectory'))
                    obj.t=varargin{1}.t;
                    obj.q=varargin{1}.q;
                    obj.qdot=varargin{1}.qdot;
                    obj.qdotdot=varargin{1}.qdotdot;
                    obj.effort=varargin{1}.effort;
                    obj.name=varargin{1}.name;
                    obj.npoints=varargin{1}.npoints;
                end
            end
            obj=obj.CompleteCartesian();
        end
        function obj = CompleteCartesian(obj)
            obj.x = [];
            obj.x = zeros(obj.npoints,6);
            for i=1:size(obj.q,1)
                obj.x(i,:)=IiwaScrewTheory.ForwardKinematics(obj.q(i,:));
            end
            obj = obj.CompleteCartesianVel();
        end
        function obj = CompleteCartesianVel(obj)
            if (obj.npoints>1)
                obj.xdot = [];
                obj.xdot = zeros(obj.npoints,6);
                for i=1:obj.npoints
                    Jst = GeoJacobianS([IiwaRobot.Twist; obj.q(i,:)]);
                    obj.xdot(i,:)=(Jst*obj.qdot(i,:)')';
                end
            end
            if (obj.npoints==1)
            	obj.xdot(obj.npoints,:)=[0 0 0 0 0 0];
            end
        end
        function obj = CompleteVelAcc(obj)
            if (isempty(obj.q))
                return;
            end
            obj.qdot = [];
            obj.qdotdot = [];
            obj.qdot = zeros(obj.npoints,7);
            obj.qdotdot = zeros(obj.npoints,7);
            css = mean(obj.t(2:end)-obj.t(1:end-1));
            sub_q = obj.q(2:end,:)-obj.q(1:end-1, :);
            sub_q = medfilt1(sub_q, round(obj.npoints/30));
            if (obj.npoints>1)
                obj.t = (0:css:css*(obj.npoints-1))';
            else
                obj.t=0;
            end
            qdot_med = sub_q./css; %Mean velocities at css/2, css/2+css, css/2 + css*2, ...
            qdot_= (qdot_med(1:end-1,:)+qdot_med(2:end,:))./2;
            obj.qdot = [zeros(1, IiwaRobot.n_joints); qdot_; zeros(1, IiwaRobot.n_joints)];
            sub_qdot = obj.qdot(2:end,:)-obj.qdot(1:end-1,:);
            sub_qdot = medfilt1(sub_qdot, round(obj.npoints/30));
            qdotdot_med = sub_qdot./css; %Mean velocities at css/2, css/2+css, css/2 + css*2, ...
            qdotdot_ = (qdotdot_med(1:end-1,:)+qdotdot_med(2:end,:))./2;
            obj.qdotdot = [zeros(1, IiwaRobot.n_joints); qdotdot_; zeros(1, IiwaRobot.n_joints)];
        end
        function obj = CompleteEffort(obj, modeID)
            %TODO:Fix
            if (strcmp(modeID,'st')==1)
               	for i = 1:size(obj.q,1)
                    tic
                    obj.effort(i,:) = IiwaScrewTheory.InverseDynamics(obj.q(i,:), obj.qdot(i,:), obj.qdotdot(i,:));
                    toc
                end
            elseif (strcmp(modeID,'matlab')==1)
                for i = 1:size(obj.q,1)
                    obj.effort(i,:) = inverseDynamics(obj.lbr, obj.q(i,:), obj.qdot(i,:), obj.qdotdot(i,:));
                end
            end
        end
        function obj = CompleteJoint(obj, qini)
            obj.q(1,:)=qini;
            for i=1:size(obj.x,1)-1
                %obj.q(i+1,:) = IiwaScrewTheory.IDK_IK_norot(obj.x(i+1,1:3), obj.q(i,:));
                obj.q(i+1,:) = IiwaScrewTheory.IDK_position(obj.x(i+1,:), obj.q(i,:), (obj.t(i+1)-obj.t(i)));
               
%                 obj.xdot(i,:) = IiwaScrewTheory.screwA2B_A(obj.x(i,:), obj.x(i+1,:))/(obj.t(i+1)-obj.t(i));
%                 xdot_S = IiwaScrewTheory.tfscrew_A2S(obj.xdot(i,:), obj.x(i,:));
%                 obj.qdot(i,:) = IiwaScrewTheory.IDK_point(obj.q(i,:), xdot_S);
%                 obj.q(i+1,:) = obj.q(i,:) + obj.qdot(i,:)*(obj.t(i+1)-obj.t(i));
            end
            obj.xdot(size(obj.x,1),:)=zeros(6,1);
            obj.qdot(size(obj.x,1),:)=zeros(size(IiwaRobot.Twist,2),1);
            obj.CompleteCartesian();
        end
        function obj = DeleteInitialEndPauses(obj)
            %Does not affect pp field, better apply before or inside bounded_spline function
            obj.CompleteCartesian();
            xdot_euc_pos=vecnorm(obj.xdot(:,2:3)');%vecnorm(obj.xdot.pos);
            min_cart_vel = obj.MinCartVelocity;%median(xdot_euc_pos); %obj.MinCartVelocity)
            indices_1=find(xdot_euc_pos>min_cart_vel);
            if (~isempty(indices_1))
                id_start=indices_1(1);
            else
                id_start=1;
            end
            indices_2=find(xdot_euc_pos>min_cart_vel/2);
            if (~isempty(indices_2))
                id_end=indices_2(end);
            else
                id_end=obj.npoints;
            end
            obj.t=obj.t(id_start:id_end);
            if (~isempty(obj.t))
                obj.t=obj.t-obj.t(1);
                obj.q=obj.q(id_start:id_end,:);
                obj.qdot=obj.qdot(id_start:id_end,:);
                %obj.qdotdot=obj.qdotdot(minindex:maxindex,:);
                obj.x=obj.x(id_start:id_end,:);
                obj.xdot=obj.xdot(id_start:id_end,:);
                if (~isempty(obj.effort))
                    obj.effort=obj.effort(id_start:id_end,:);
                end
                obj.npoints = id_end-id_start+1;
            end
        end
        function obj = ChangeSampleTime(obj, sample_time)
            q_ts = timeseries(obj.q, obj.t);
            q_ts_new = q_ts.resample(0:sample_time:obj.t(end));
            obj.t =  q_ts_new.Time;
            obj.q = q_ts_new.Data;
            obj.npoints = length(obj.t);
            obj = obj.CompleteVelAcc();
            obj = obj.CompleteCartesian();
        end
        function [obj, C, R, arc, w, e_rms] = FitToCircle(obj, fixed_plane, display)
            %fixed_plane = 'X', 'Y' or 'Z'
            if (~(strcmp(fixed_plane, 'X')==1 || strcmp(fixed_plane, 'Y')==1 || strcmp(fixed_plane, 'Z')))
                MException('IiwaTrajectory:FitToCircleWrongPlane', 'Fixed_plane must be a string "X", "Y" or "Z"');
            end
            while(~isempty(display.Children))
                delete(display.Children(1))
            end
            ax = axes(display);
            coord_names=['X', 'Y', 'Z'];
            %Find points in the plane used
            coords_used = [1 2 3];
            coord_fixed = strfind(coord_names, fixed_plane);
            coords_used(coord_fixed)=[];
            points3D = obj.x(1:ceil(obj.npoints/obj.PointsForCircle):end, 1:3);
            points2D = points3D(:, coords_used);
            [C, R, plots] = IiwaTrajectory.CircleFitByPerps(points2D);
            %[C, R] = IiwaTrajectory.CircleFitByPratt(points2D);
            %Find intersection between circle and C-p1 and C-pend lines
            [pstart, pend] = IiwaTrajectory.GetCircleCloserPositions(obj.x(1,coords_used), obj.x(end,coords_used), R, C);
            %Fill the arc
            A = [pstart(1); pstart(2)];
            B = [pend(1); pend(2)];
            a = atan2(A(2)-C(2),A(1)-C(1));
            b = atan2(B(2)-C(2),B(1)-C(1));
            b = mod(b-a,2*pi)+a; % Ensure that arc moves counterclockwise
            t_ = linspace(a,b,obj.npoints)';
            XY = [(C(1)+R*cos(t_)) (C(2)+R*sin(t_))];
            %Plot
            plot(ax, obj.x(:,coords_used(1)), obj.x(:,coords_used(2)), 'LineWidth', 3, 'Color', [0.47, 0.67, 0.19]);
            hold(ax, 'on');
            plot(ax, XY(:,1), XY(:,2), 'LineWidth', 3, 'Color', [0, 0.45, 0.74]);
            plot(ax, C(1), C(2), '*', 'MarkerSize', 10, 'LineWidth', 3, 'Color', [0, 0.45, 0.74]);
            for i=1:size(plots,1)
                plot(ax, plots(i,1:2), plots(i,3:4));
                hold(ax, 'on');
            end
            axis(ax, 'equal');
            legend(ax, 'captured', 'circle fit');
            xlabel(ax, 'Y');
            ylabel(ax, 'Z');
            d_e = XY-obj.x(:,coords_used);
            e_rms = rms(vecnorm(d_e'));
            title(ax, ['Error: ', num2str(e_rms), ' cm']);
            
            %Find circunference angle and velocity
            u = [0 pstart-[C(1) C(2)]];
            v = [0 pend-[C(1) C(2)]];
            arc = rad2deg(atan2(norm(cross(u,v)), dot(u,v)));
            w = arc/obj.t(end);
            %Fill output
            obj.x(:, coords_used)=XY;
            obj.x(:, coord_fixed)=mean(points3D(:,coord_fixed));
            obj.x(:, coords_used+3)=ones(size(obj.x(:,coords_used+3))).*obj.x(1,coords_used+3);
            q_ini = IiwaScrewTheory.IDK_IK_norot(obj.x(1,:), obj.q(1,:));
            obj = obj.CompleteJoint(q_ini);
            obj = obj.CompleteVelAcc();
            obj = obj.CompleteCartesianVel();
        end
        function obj = AddPause(obj, t_pause)
            sample_time = mean(obj.t(2:end)-obj.t(1:end-1));
            obj.t = [obj.t; (obj.t(end)+sample_time:sample_time:obj.t(end)+t_pause)'];
            npoints_added = length(obj.t)-obj.npoints;
            obj.npoints = length(obj.t);
            obj.q = [obj.q; repmat(obj.q(end,:), npoints_added, 1)];
            obj.x = [obj.x; repmat(obj.x(end,:), npoints_added, 1)];
            obj.qdot = [obj.qdot; repmat([0 0 0 0 0 0 0], npoints_added, 1)];
            obj.qdotdot = [obj.qdotdot; repmat([0 0 0 0 0 0 0], npoints_added, 1)];
            obj.xdot = [obj.xdot; repmat([0 0 0 0 0 0 ], npoints_added, 1)];
            obj.xdotdot = [obj.xdotdot; repmat([0 0 0 0 0 0 ], npoints_added, 1)];
        end
        function obj = AddPauseBefore(obj, t_pause)
            sample_time = mean(obj.t(2:end)-obj.t(1:end-1));
            obj.t = [(0:sample_time:t_pause)'; obj.t+t_pause];
            npoints_added = length(obj.t)-obj.npoints;
            obj.npoints = length(obj.t);
            obj.q = [repmat(obj.q(1,:), npoints_added, 1); obj.q];
            obj.x = [repmat(obj.x(1,:), npoints_added, 1); obj.x];
            obj.qdot = [repmat([0 0 0 0 0 0 0], npoints_added, 1); obj.qdot];
            obj.qdotdot = [repmat([0 0 0 0 0 0 0], npoints_added, 1); obj.qdotdot];
            obj.xdot = [repmat([0 0 0 0 0 0 ], npoints_added, 1); obj.xdot];
            obj.xdotdot = [repmat([0 0 0 0 0 0 ], npoints_added, 1); obj.xdotdot];
        end
        function obj = MirrorTrajectory(obj)
            obj.q = flipud(obj.q);
            obj = obj.CompleteVelAcc();
            obj = obj.CompleteCartesian();
        end
        function obj = MergeAfterwards(obj, traj_added)
            obj.npoints = obj.npoints + traj_added.npoints;
            obj.q = [obj.q; traj_added.q];
            obj.x = [obj.x; traj_added.x];
            obj.qdot = [obj.qdot; traj_added.qdot];
            obj.xdot = [obj.xdot; traj_added.xdot];
            obj.xdotdot = [obj.xdotdot; traj_added.xdotdot];
            obj.qdotdot = [obj.qdotdot; traj_added.qdotdot];
            obj.t = [obj.t; obj.t(end)+traj_added.t+mean(mean(traj_added.t(2:end)-traj_added.t(1:end-1)))];
        end
        function obj = ChangeVelocity(obj, v)
            tsample = mean(obj.t(2:end)-obj.t(1:end-1));
            obj.t = obj.t./v;
            obj = obj.CompleteVelAcc();
            obj = obj.CompleteCartesianVel();
            obj = obj.ChangeSampleTime(tsample);
        end
        function obj = BoundedSpline(obj, smoothing, nSegments)
            if (isempty(obj.t))
                disp('IiwaTrajectoryGeneration:BoundedSplineTrajectory:Empty time vector');
                return 
            end
            t_ = obj.t';
            q_ = obj.q';
            nKnots=nSegments+1;
            tKnot=linspace(t_(1), t_(end), nKnots);    

            %% Run main function
            [H, f] = IiwaTrajectory.computeQuadraticCostMatrix(t_', q_', tKnot', smoothing);
            nState = size(q_,1);
            [A1, b1] = IiwaTrajectory.computeEqCstContinuity(tKnot, nState);
            lowBnd.pos = q_(:,1)';
            lowBnd.vel = zeros(1, size(q_,1));
            lowBnd.acc = zeros(1, size(q_,1)); %[];% 
            uppBnd.pos = q_(:,end)';
            uppBnd.vel = zeros(1, size(q_,1));
            uppBnd.acc = zeros(1, size(q_,1)); %[];% 
            [A2, b2] = IiwaTrajectory.computeEqCstBoundary(lowBnd, uppBnd, tKnot);
            coeff = IiwaTrajectory.solveQpEq(H,f,[A1;A2],[b1;b2]);
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
            qdotpp = IiwaTrajectory.ppDer(qpp);
            qdotdotpp = IiwaTrajectory.ppDer(qdotpp);
            % Fill output
%             %%CSAPE
%             tPoints = length(obj.t);
%             nPoints = nSegments+1;
%             x_ = obj.t(1:round(tPoints/nPoints):end); 
%             y_ = obj.q(1:round(tPoints/nPoints):end,:)';
% 
%             pp3 = csape(x_,y_, 'complete', [0 0 0 0 0 0 0; 0 0 0 0 0 0 0]');
%             dpp3 = fnder(pp3,1);
%             ddpp3 = fnder(dpp3,1);
% 
%             q_pp = ppval(pp3,obj.t)';
%             qd_pp = ppval(dpp3, obj.t)';
%             qdd_pp = ppval(ddpp3, obj.t)';
% 
%             q_knot = ppval(pp3, x_)';
%             qd_knot = ppval(dpp3, x_)';
%             qdd_knot = ppval(ddpp3, x_)';
%             
%             obj.name = 'spline';
%             obj.q = q_pp;
%             obj.qdot = qd_pp;
%             obj.qdotdot = qdd_pp;
%             obj = obj.CompleteCartesian();
  
            obj = IiwaTrajectory('spline', length(t_));
            obj.t=t_';
            obj.q=(ppval(qpp, t_))';
            obj.qdot=(ppval(qdotpp, t_))';
            obj.qdotdot=(ppval(qdotdotpp, t_))';
            obj = obj.CompleteCartesian();
%             Copyright (c) 2017, Matthew Kelly
%             All rights reserved.
% 
%             Redistribution and use in source and binary forms, with or without
%             modification, are permitted provided that the following conditions are met:
% 
%             * Redistributions of source code must retain the above copyright notice, this
%               list of conditions and the following disclaimer.
% 
%             * Redistributions in binary form must reproduce the above copyright notice,
%               this list of conditions and the following disclaimer in the documentation
%               and/or other materials provided with the distribution
%             THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%             AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%             IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
%             DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
%             FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
%             DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
%             SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
%             CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
%             OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
%             OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
        end
        function obj = GetFirstNPoints(obj, npoints)
            obj.t = obj.t(1:npoints);
            obj.q = obj.q(1:npoints,:);
            obj.x = obj.x(1:npoints,:);
            obj.qdot = obj.qdot(1:npoints,:);
            obj.qdotdot = obj.qdotdot(1:npoints,:);
            obj.xdot = obj.xdot(1:npoints,:);
            obj.npoints = npoints;
        end
        function obj = FixCartesianCoordinates(obj, coordinates)
            %Does not modify joint coordinates
        	if (contains(coordinates, 'X'))
                obj.x(:,1) = ones(obj.npoints,1).*obj.x(1,1);
                obj.xdot(:,1) = zeros(size(obj.xdot(:,1)));
            end
            if (contains(coordinates, 'Y'))
                obj.x(:,2) = ones(obj.npoints,1).*obj.x(1,2);
                obj.xdot(:,2) = zeros(size(obj.xdot(:,2)));
            end
            if (contains(coordinates, 'Z'))
                obj.x(:,3) = ones(obj.npoints,1).*obj.x(1,3);
                obj.xdot(:,3) = zeros(size(obj.xdot(:,3)));
            end
            if (contains(coordinates, 'A'))
                obj.x(:,4) = ones(obj.npoints,1).*obj.x(1,4);
                obj.xdot(:,4) = zeros(size(obj.xdot(:,4)));
            end
            if (contains(coordinates, 'B'))
                obj.x(:,5) = ones(obj.npoints,1).*obj.x(1,5);
                obj.xdot(:,5) = zeros(size(obj.xdot(:,5)));
            end
            if (contains(coordinates, 'C'))
                obj.x(:,6) = ones(obj.npoints,1).*obj.x(1,6);
                obj.xdot(:,6) = zeros(size(obj.xdot(:,6)));
            end            
        end
    end
    methods (Access=private)
        function obj = GetCartRotVel(obj)
            ori_vec = obj.x(:,4:6);
            t_vec = obj.t;
            ori_dot_med=zeros(size(ori_vec,1)-1, 3);
            for i=1:size(ori_vec,1)-1
                time = t_vec(i+1)-t_vec(i);
                axang3_AB_A= IiwaScrewTheory.axangA2B_A(ori_vec(i,:), ori_vec(i+1,:));%axang4_AB_A(1:3)*axang4_AB_A(4); %[axis_x*angle, axis_y*angle, axis_z*angle]
                %Express this rotation in the space frame, as the previous was
                %expressed in the A frame, just multiply by R_SA
                R_SA=eul2rotm(ori_vec(i,:), 'XYZ');
                axang3_AB_S=R_SA*axang3_AB_A';
                %Divide by the time
                ori_dot_med(i,:)=axang3_AB_S'/time;
            end
            ori_dot = (ori_dot_med(1:end-1,:)+ori_dot_med(2:end,:))/2;
            obj.xdot(2:end-1,4:6) = ori_dot;
        end      
    end
    methods(Static, Access=public)
        function [traj_min, traj_mean, traj_max, traj_min_error, traj_mean_error, traj_max_error, traj_min_error_rep, traj_mean_error_rep, traj_max_error_rep] = MinMeanMaxTrajectories(trajectories, traj_command)
            %GetSampleTime of each of them
            for i_traj=1:size(trajectories,2)
                traj_i = trajectories{i_traj};
                st(i_traj)=mean(traj_i.t(2:end)-traj_i.t(1:end-1));
            end
            st(i_traj+1)=mean(traj_command.t(2:end)-traj_command.t(1:end-1));
            sample_time_used = mean(st);
            %Normalize all of them to the mean sample time, save npoints
            for i_traj=1:size(trajectories,2)
                trajectories{i_traj} = trajectories{i_traj}.ChangeSampleTime(sample_time_used);
                npoints(i_traj)=trajectories{i_traj}.npoints;
            end
            traj_command = traj_command.ChangeSampleTime(sample_time_used);
            npoints(i_traj+1) = traj_command.npoints;
            npoints_used = min(npoints);
            %Set to the same amount of points to allign them
            for i_traj=1:size(trajectories,2)
                trajs{i_traj} = trajectories{i_traj}.GetFirstNPoints(npoints_used);
            end
            traj_comm= traj_command.GetFirstNPoints(npoints_used);
            %Allign command with first captured trajectory
            for d=1:round(1/sample_time_used)    
                e(d) = mean(vecnorm((trajs{1}.q(d:end,:)-traj_comm.q(1:end-d+1,:))'));
            end
            [m, id] = min(e);
            traj_command = traj_command.AddPauseBefore(id*sample_time_used);
            %Delete points not used
            for i_traj=1:size(trajectories,2)
                npoints(i_traj)=trajectories{i_traj}.npoints;
            end
            npoints(i_traj+1) = traj_command.npoints;
            npoints_used = min(npoints);
            %Set to the same amount of points to allign them
            for i_traj=1:size(trajectories,2)
                trajectories{i_traj} = trajectories{i_traj}.GetFirstNPoints(npoints_used);
            end
            traj_command= traj_command.GetFirstNPoints(npoints_used);

            %Get minimum and maximum trajectory
            traj_min = IiwaTrajectory('min', npoints_used);
            traj_max = IiwaTrajectory('max', npoints_used);
            traj_mean = IiwaTrajectory('mean', npoints_used);
            traj_min_error = IiwaTrajectory('min_error', npoints_used);
            traj_max_error = IiwaTrajectory('max_error', npoints_used);
            traj_mean_error = IiwaTrajectory('mean_error', npoints_used);
            
            traj_min.t = (0:sample_time_used:(npoints_used-1)*sample_time_used)';
            traj_max.t = (0:sample_time_used:(npoints_used-1)*sample_time_used)';
            traj_mean.t = (0:sample_time_used:(npoints_used-1)*sample_time_used)';
            traj_min_error.t = (0:sample_time_used:(npoints_used-1)*sample_time_used)';
            traj_max_error.t = (0:sample_time_used:(npoints_used-1)*sample_time_used)';
            traj_mean_error.t = (0:sample_time_used:(npoints_used-1)*sample_time_used)';           
            
            traj_min.q = inf(size(traj_min.q));
            traj_min.qdot = inf(size(traj_min.qdot));
            traj_min.qdotdot = inf(size(traj_min.qdotdot));
            traj_min.x = inf(size(traj_min.x));
            traj_min.xdot = inf(size(traj_min.xdot));
            
            traj_min_error.q = inf(size(traj_min_error.q));
            traj_min_error.qdot = inf(size(traj_min_error.qdot));
            traj_min_error.qdotdot = inf(size(traj_min_error.qdotdot));
            traj_min_error.x = inf(size(traj_min_error.x));
            traj_min_error.xdot = inf(size(traj_min_error.xdot));
            
            traj_mean.q = zeros(size(traj_mean.q));
            traj_mean.qdot = zeros(size(traj_mean.qdot));
            traj_mean.qdotdot = zeros(size(traj_mean.qdotdot));
            traj_mean.x = zeros(size(traj_mean.x));
            traj_mean.xdot = zeros(size(traj_mean.xdot));
            
            traj_mean_error.q = zeros(size(traj_mean_error.q));
            traj_mean_error.qdot = zeros(size(traj_mean_error.qdot));
            traj_mean_error.qdotdot = zeros(size(traj_mean_error.qdotdot));
            traj_mean_error.x = zeros(size(traj_mean_error.x));
            traj_mean_error.xdot = zeros(size(traj_mean_error.xdot));

            traj_max.q = -inf(size(traj_max.q));
            traj_max.qdot = -inf(size(traj_max.qdot));
            traj_max.qdotdot = -inf(size(traj_max.qdotdot));
            traj_max.x = -inf(size(traj_max.x));
            traj_max.xdot = -inf(size(traj_max.xdot));
            
            traj_max_error.q = -inf(size(traj_max_error.q));
            traj_max_error.qdot = -inf(size(traj_max_error.qdot));
            traj_max_error.qdotdot = -inf(size(traj_max_error.qdotdot));
            traj_max_error.x = -inf(size(traj_max_error.x));
            traj_max_error.xdot = -inf(size(traj_max_error.xdot));
                        
            for i_traj = 1:size(trajectories,2)
                traj_min.q = min(traj_min.q, trajectories{i_traj}.q);
                traj_min.qdot = min(traj_min.qdot, trajectories{i_traj}.qdot);
                traj_min.qdotdot= min(traj_min.qdotdot, trajectories{i_traj}.qdotdot);
                traj_min.x = min(traj_min.x, trajectories{i_traj}.x);
                traj_min.xdot = min(traj_min.xdot, trajectories{i_traj}.xdot);
                
                traj_max.q = max(traj_max.q, trajectories{i_traj}.q);
                traj_max.qdot = max(traj_max.qdot, trajectories{i_traj}.qdot);
                traj_max.qdotdot = max(traj_max.qdotdot, trajectories{i_traj}.qdotdot);
                traj_max.x = max(traj_max.x, trajectories{i_traj}.x);
                traj_max.xdot = max(traj_max.xdot, trajectories{i_traj}.xdot);
                
                traj_mean.q = traj_mean.q + trajectories{i_traj}.q;
                traj_mean.qdot = traj_mean.qdot + trajectories{i_traj}.qdot;
                traj_mean.qdotdot = traj_mean.qdotdot + trajectories{i_traj}.qdotdot;
                traj_mean.x = traj_mean.x + trajectories{i_traj}.x;
                traj_mean.xdot = traj_mean.xdot + trajectories{i_traj}.xdot;
                                
                traj_min_error.q = min(traj_min_error.q, abs(traj_command.q-trajectories{i_traj}.q));
                traj_min_error.qdot = min(traj_min_error.qdot, abs(traj_command.qdot-trajectories{i_traj}.qdot));
                traj_min_error.qdotdot = min(traj_min_error.qdotdot, abs(traj_command.qdotdot-trajectories{i_traj}.qdotdot));
                traj_min_error.x = min(traj_min_error.x, abs(traj_command.x-trajectories{i_traj}.x));
                traj_min_error.xdot = min(traj_min_error.xdot, abs(traj_command.xdot-trajectories{i_traj}.xdot));
                
                traj_max_error.q = max(traj_max_error.q, abs(traj_command.q-trajectories{i_traj}.q));
                traj_max_error.qdot = max(traj_max_error.qdot, abs(traj_command.qdot-trajectories{i_traj}.qdot));
                traj_max_error.qdotdot = max(traj_max_error.qdotdot, abs(traj_command.qdotdot-trajectories{i_traj}.qdotdot));
                traj_max_error.x = max(traj_max_error.x, abs(traj_command.x-trajectories{i_traj}.x));
                traj_max_error.xdot = max(traj_max_error.xdot, abs(traj_command.xdot-trajectories{i_traj}.xdot));

                traj_mean_error.q = traj_mean_error.q + abs(traj_command.q-trajectories{i_traj}.q);
                traj_mean_error.qdot = traj_mean_error.qdot + abs(traj_command.qdot-trajectories{i_traj}.qdot);
                traj_mean_error.qdotdot = traj_mean_error.qdotdot + abs(traj_command.qdotdot-trajectories{i_traj}.qdotdot);
                traj_mean_error.x = traj_mean_error.x + abs(traj_command.x-trajectories{i_traj}.x);
                traj_mean_error.xdot = traj_mean_error.xdot + abs(traj_command.xdot-trajectories{i_traj}.xdot);
                
            end
            
            traj_mean.q = traj_mean.q./size(trajectories,2);
            traj_mean.qdot = traj_mean.qdot./size(trajectories,2);
            traj_mean.qdotdot = traj_mean.qdotdot./size(trajectories,2);
            traj_mean.x = traj_mean.x./size(trajectories,2);
            traj_mean.xdot = traj_mean.xdot./size(trajectories,2);
            
            traj_mean_error.q = traj_mean_error.q./size(trajectories,2);
            traj_mean_error.qdot = traj_mean_error.qdot./size(trajectories,2);
            traj_mean_error.qdotdot = traj_mean_error.qdotdot./size(trajectories,2);
            traj_mean_error.x = traj_mean_error.x./size(trajectories,2);
            traj_mean_error.xdot = traj_mean_error.xdot./size(trajectories,2);
            
            
            %Now compare each trial with the mean trial - Repeatibility
            traj_min_error_rep = IiwaTrajectory('min_error_rep', npoints_used);
            traj_mean_error_rep = IiwaTrajectory('mean_error_rep', npoints_used);
            traj_max_error_rep = IiwaTrajectory('max_error_rep', npoints_used);
            
            traj_min_error_rep.t = (0:sample_time_used:(npoints_used-1)*sample_time_used)';
            traj_mean_error_rep.t = (0:sample_time_used:(npoints_used-1)*sample_time_used)';
            traj_max_error_rep.t = (0:sample_time_used:(npoints_used-1)*sample_time_used)';
            
            traj_min_error_rep.q = inf(size(traj_min_error_rep.q));
            traj_min_error_rep.qdot = inf(size(traj_min_error_rep.qdot));
            traj_min_error_rep.qdotdot = inf(size(traj_min_error_rep.qdotdot));
            traj_min_error_rep.x = inf(size(traj_min_error_rep.x));
            traj_min_error_rep.xdot = inf(size(traj_min_error_rep.xdot));
            
            traj_mean_error_rep.q = zeros(size(traj_mean_error_rep.q));
            traj_mean_error_rep.qdot = zeros(size(traj_mean_error_rep.qdot));
            traj_mean_error_rep.qdotdot = zeros(size(traj_mean_error_rep.qdotdot));
            traj_mean_error_rep.x = zeros(size(traj_mean_error_rep.x));
            traj_mean_error_rep.xdot = zeros(size(traj_mean_error_rep.xdot));
            
            traj_max_error_rep.q = -inf(size(traj_max_error_rep.q));
            traj_max_error_rep.qdot = -inf(size(traj_max_error_rep.qdot));
            traj_max_error_rep.qdotdot = -inf(size(traj_max_error_rep.qdotdot));
            traj_max_error_rep.x = -inf(size(traj_max_error_rep.x));
            traj_max_error_rep.xdot = -inf(size(traj_max_error_rep.xdot));
            
            for i_traj = 1:size(trajectories,2)
                traj_mean_error_rep.q = traj_mean_error_rep.q + abs(traj_mean.q-trajectories{i_traj}.q);
                traj_mean_error_rep.qdot = traj_mean_error_rep.qdot + abs(traj_mean.qdot-trajectories{i_traj}.qdot);
                traj_mean_error_rep.qdotdot = traj_mean_error_rep.qdotdot + abs(traj_mean.qdotdot-trajectories{i_traj}.qdotdot);
                traj_mean_error_rep.x = traj_mean_error_rep.x + abs(traj_mean.x-trajectories{i_traj}.x);
                traj_mean_error_rep.xdot = traj_mean_error_rep.xdot + abs(traj_mean.xdot-trajectories{i_traj}.xdot);
                
                traj_min_error_rep.q = min(traj_min_error_rep.q, abs(traj_mean.q-trajectories{i_traj}.q));
                traj_min_error_rep.qdot = min(traj_min_error_rep.qdot, abs(traj_mean.qdot-trajectories{i_traj}.qdot));
                traj_min_error_rep.qdotdot = min(traj_min_error_rep.qdotdot, abs(traj_mean.qdotdot-trajectories{i_traj}.qdotdot));
                traj_min_error_rep.x = min(traj_min_error_rep.x, abs(traj_mean.x-trajectories{i_traj}.x));
                traj_min_error_rep.xdot = min(traj_min_error_rep.xdot, abs(traj_mean.xdot-trajectories{i_traj}.xdot));
                                
                traj_max_error_rep.q = max(traj_max_error_rep.q, abs(traj_mean.q-trajectories{i_traj}.q));
                traj_max_error_rep.qdot = max(traj_max_error_rep.qdot, abs(traj_mean.qdot-trajectories{i_traj}.qdot));
                traj_max_error_rep.qdotdot = max(traj_max_error_rep.qdotdot, abs(traj_mean.qdotdot-trajectories{i_traj}.qdotdot));
                traj_max_error_rep.x = max(traj_max_error_rep.x, abs(traj_mean.x-trajectories{i_traj}.x));
                traj_max_error_rep.xdot = max(traj_max_error_rep.xdot, abs(traj_mean.xdot-trajectories{i_traj}.xdot));
            end
            traj_mean_error_rep.q = traj_mean_error_rep.q./size(trajectories,2);
            traj_mean_error_rep.qdot = traj_mean_error_rep.qdot./size(trajectories,2);
            traj_mean_error_rep.qdotdot = traj_mean_error_rep.qdotdot./size(trajectories,2);
            traj_mean_error_rep.x = traj_mean_error_rep.x./size(trajectories,2);
            traj_mean_error_rep.xdot = traj_mean_error_rep.xdot./size(trajectories,2);
            
            traj_min_error.x(:,4:6)=wrapToPi(traj_min_error.x(:,4:6));
            traj_mean_error.x(:,4:6)=wrapToPi(traj_mean_error.x(:,4:6));     
            traj_max_error.x(:,4:6)=wrapToPi(traj_max_error.x(:,4:6));
            traj_min_error_rep.x(:,4:6)=wrapToPi(traj_min_error_rep.x(:,4:6));
            traj_mean_error_rep.x(:,4:6)=wrapToPi(traj_mean_error_rep.x(:,4:6));     
            traj_max_error_rep.x(:,4:6)=wrapToPi(traj_max_error_rep.x(:,4:6));
            
            
            traj_mean_error.x(abs(traj_mean_error.x(:,4:6))>0.1745,4:6)=0;
            traj_min_error.x(abs(traj_min_error.x(:,4:6))>0.1745,4:6)=0;
            traj_max_error.x(abs(traj_max_error.x(:,4:6))>0.1745,4:6)=0;
            traj_mean_error_rep.x(abs(traj_mean_error_rep.x(:,4:6))>0.1745,4:6)=0;
            traj_min_error_rep.x(abs(traj_min_error_rep.x(:,4:6))>0.1745,4:6)=0;
            traj_max_error_rep.x(abs(traj_max_error_rep.x(:,4:6))>0.1745,4:6)=0;
            
            traj_mean_error.x=traj_mean_error.x(1:npoints_used,:);
            traj_min_error.x=traj_min_error.x(1:npoints_used,:);
            traj_max_error.x=traj_max_error.x(1:npoints_used,:);
            traj_mean_error_rep.x=traj_mean_error_rep.x(1:npoints_used,:);
            traj_min_error_rep.x=traj_min_error_rep.x(1:npoints_used,:);
            traj_max_error_rep.x=traj_max_error_rep.x(1:npoints_used,:);
        end
    end
    methods (Static, Access=private)
        function [C, R, plots] = CircleFitByPerps(XY)
            %% Find segments and perpendicular lines
            plots = [];
            margin=0.10;
            xmin=min(XY(:,1))-margin;
            xmax=max(XY(:,1))+margin;
            ymin=min(XY(:,2))-margin;
            ymax=max(XY(:,2))+margin;
            x_=linspace(xmin, xmax, 1000);
            for i=1:size(XY,1)-1
                %Plot segment
                seg.x=[XY(i,1), XY(i+1,1)];
                seg.y=[XY(i,2), XY(i+1,2)];
                B=[ones(size(seg.x')) seg.x']\seg.y';
                seg.intercept=B(1);
                seg.slope=B(2);
                seg.center.x=mean(seg.x);
                seg.center.y=mean(seg.y);
                plots = [plots; [seg.x, seg.y]];
                %Find perpendicular line between (xmin, xmax)
                y_=-1/seg.slope*(x_-seg.center.x)+seg.center.y;
                in_axis=(max(find(y_>=ymin & y_<=ymax, 1, 'first')-1,1):min(find(y_>=ymin & y_<=ymax, 1, 'last')+1,size(y_,2)));
                if (isempty(in_axis))
                    %in_axis=(max(find(x>=xmin & x<=xmax, 1, 'first')-1,1):min(find(x>=xmin & x<=xmax, 1, 'last')+1,size(x,2)));
                    ME=MException('Adjust:TODO', 'To do: Treat with horizontal lines');
                    throw(ME);
                end
                xx=x_(in_axis);
                yy=y_(in_axis);
                perp.x=[xx(1), xx(end)];
                perp.y=[yy(1), yy(end)];
                B=[ones(size(perp.x')) perp.x']\perp.y';
                perp.intercept=B(1);
                perp.slope=B(2);
                plots = [plots; [perp.x, perp.y]];
                segs(i)=seg;
                perps(i)=perp;
            end

            %% Find intersections
            intersections=[];
            for i=1:size(perps,2)
                for j=1:size(perps,2)
                    if (i~=j)
                        [x_,y_]=polyxpoly(perps(i).x, perps(i).y, perps(j).x, perps(j).y);
                        intersections=[intersections; x_ y_];
                    end
                end
            end
            if (isempty(intersections))
                C=XY(1,:);
                R=0.01;
            else
            %% Find center and radius of circumference
            C=mean(intersections);
            %% Find radius of circumference
            R=mean(vecnorm((XY-C)'));
            end
        end
        function [C, R] = CircleFitByPratt(XY)
        %--------------------------------------------------------------------------
        %  
        %     Circle fit by Pratt
        %      V. Pratt, "Direct least-squares fitting of algebraic surfaces",
        %      Computer Graphics, Vol. 21, pages 145-152 (1987)
        %
        %     Input:  XY(n,2) is the array of coordinates of n points x(i)=XY(i,1), y(i)=XY(i,2)
        %
        %     Output: Par = [a b R] is the fitting circle:
        %                           center (a,b) and radius R
        %
        %     Note: this fit does not use built-in matrix functions (except "mean"),
        %           so it can be easily programmed in any programming language
        %
        %--------------------------------------------------------------------------
        n = size(XY,1);      % number of data points
        centroid = mean(XY);   % the centroid of the data set
        %     computing moments (note: all moments will be normed, i.e. divided by n)
        Mxx=0; Myy=0; Mxy=0; Mxz=0; Myz=0; Mzz=0;
        for i=1:n
            Xi = XY(i,1) - centroid(1);  %  centering data
            Yi = XY(i,2) - centroid(2);  %  centering data
            Zi = Xi*Xi + Yi*Yi;
            Mxy = Mxy + Xi*Yi;
            Mxx = Mxx + Xi*Xi;
            Myy = Myy + Yi*Yi;
            Mxz = Mxz + Xi*Zi;
            Myz = Myz + Yi*Zi;
            Mzz = Mzz + Zi*Zi;
        end

        Mxx = Mxx/n;
        Myy = Myy/n;
        Mxy = Mxy/n;
        Mxz = Mxz/n;
        Myz = Myz/n;
        Mzz = Mzz/n;
        %    computing the coefficients of the characteristic polynomial
        Mz = Mxx + Myy;
        Cov_xy = Mxx*Myy - Mxy*Mxy;
        Mxz2 = Mxz*Mxz;
        Myz2 = Myz*Myz;
        A2 = 4*Cov_xy - 3*Mz*Mz - Mzz;
        A1 = Mzz*Mz + 4*Cov_xy*Mz - Mxz2 - Myz2 - Mz*Mz*Mz;
        A0 = Mxz2*Myy + Myz2*Mxx - Mzz*Cov_xy - 2*Mxz*Myz*Mxy + Mz*Mz*Cov_xy;
        A22 = A2 + A2;
        epsilon=1e-12; 
        ynew=1e+20;
        IterMax=20;
        xnew = 0;
        %    Newton's method starting at x=0
        for iter=1:IterMax
            yold = ynew;
            ynew = A0 + xnew*(A1 + xnew*(A2 + 4.*xnew*xnew));
            if (abs(ynew)>abs(yold))
                disp('Newton-Pratt goes wrong direction: |ynew| > |yold|');
                xnew = 0;
                break;
            end
            Dy = A1 + xnew*(A22 + 16*xnew*xnew);
            xold = xnew;
            xnew = xold - ynew/Dy;
            if (abs((xnew-xold)/xnew) < epsilon), break, end
            if (iter >= IterMax)
                disp('Newton-Pratt will not converge');
                xnew = 0;
            end
            if (xnew<0.)
                fprintf(1,'Newton-Pratt negative root:  x=%f\n',xnew);
                xnew = 0;
            end
        end
        %    computing the circle parameters
        DET = xnew*xnew - xnew*Mz + Cov_xy;
        Center = [Mxz*(Myy-xnew)-Myz*Mxy , Myz*(Mxx-xnew)-Mxz*Mxy]/DET/2;
        %Par = [Center+centroid , sqrt(Center*Center'+Mz+2*xnew)];
        C = Center+centroid;
        R = sqrt(Center*Center'+Mz+2*xnew);
        end
        function [pstart_circ, pend_circ] = GetCircleCloserPositions(pstart, pend, R, C)
            x_p1_C = [pstart(1), C(1)];
            y_p1_C = [pstart(2), C(2)];
            B = [ones(size(x_p1_C')), x_p1_C']\y_p1_C';
            intercept_pstart_C = B(1);
            slope_pstart_C = B(2);
            x_pend_C = [pend(1), C(1)];
            y_pend_C = [pend(2), C(2)];
            B = [ones(size(x_pend_C')), x_pend_C']\y_pend_C';
            intercept_pend_C = B(1);
            slope_pend_C = B(2);
            [xstart, ystart] = linecirc(slope_pstart_C, intercept_pstart_C, C(1), C(2), R);
            [xend, yend] = linecirc(slope_pend_C, intercept_pend_C, C(1), C(2), R);
            %Select solution, as there are 2 per line
            xystart = [xstart; ystart];
            xyend = [xend; yend];
            dstart = vecnorm(xystart-pstart');
            [~, idstart] = min(dstart);
            pstart_circ = xystart(:,idstart)';
            dend = vecnorm(xyend-pend');
            [~, idend] = min(dend);
            pend_circ = xyend(:,idend)';
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
                   [hTmp, fTmp] = IiwaTrajectory.singleDataPoint(tBin(iData) - tZero, xBin(iData,:));
                 H(idx, idx) = H(idx, idx) +  hTmp;
                 f(idx,:) = f(idx,:) + fTmp;
               end
               hSeg = tKnot(iBin+1) - tKnot(iBin);
               H(idx, idx) = beta*H(idx, idx) + smooth*IiwaTrajectory.minJerk(hSeg);
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
              [upp, low] = IiwaTrajectory.continuityConstraint(hSeg);
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

