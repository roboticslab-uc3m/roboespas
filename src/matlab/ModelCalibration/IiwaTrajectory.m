classdef IiwaTrajectory < handle
    %IIWAINTERPOLATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access=public)
        t
        q
        qdot
        qdotdot
        effort
        name
    end
    properties (Access=private)
        iiwa_robot
    end
    
    methods
        
        function obj = IiwaTrajectory(varargin) %iiwa_robot, name, tWaypoints, qWaypoints, t)
            if (length(varargin)==5)
                obj=obj.wayPointsConstructor(varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5});
            elseif (length(varargin)==2)
                obj=IiwaMsgTransformer.toIiwaTrajectory(varargin{1}, varargin{2});
            end
        end
        function obj = wayPointsConstructor(obj, iiwa_robot, name, tWaypoints, qWaypoints, t)
            %Creates a trajectory that passes through some joint positions
            %at certain timestamps  
            ndof = size(qWaypoints,2);
            dt = t(end)-t(end-1);
            Q = [qWaypoints(1,:);  qWaypoints; qWaypoints(end,:)]';
            T = [0, tWaypoints+dt, tWaypoints(end)+2*dt] ;
            obj.t = [0, t + dt, t(end)+2*dt]';

            pp_pos = pchip(T,Q);
            pieces = pp_pos.pieces;

            coefs_vel = zeros(ndof*pieces,3); 
            coefs_acc = zeros(ndof*pieces,2);
            for k = 1:ndof
                for i = 1:pieces
                    c = pp_pos.coefs((k-1)*pieces+i,:);
                    coefs_vel((k-1)*pieces+i,:) = [3*c(1) 2*c(2) c(3)];
                    coefs_acc((k-1)*pieces+i,:) = [6*c(1) 2*c(2)];
                end
            end

            pp_vel = pp_pos;
            pp_vel.order = 3;
            pp_vel.coefs = coefs_vel; 

            pp_acc = pp_pos;
            pp_acc.order = 2;
            pp_acc.coefs = coefs_acc;

            obj.q = ppval(pp_pos, obj.t)';
            obj.qdot = ppval(pp_vel, obj.t)';
            obj.qdotdot = ppval(pp_acc, obj.t)';
            obj.name=name;
            obj.effort = zeros(size(obj.q,1),7);
           
            for i = 1:size(obj.q,1)
                obj.effort(i,:) = inverseDynamics(iiwa_robot, obj.q(i,:), obj.qdot(i,:), obj.qdotdot(i,:));
            end
            obj.iiwa_robot=iiwa_robot;
        end 
    end
end

