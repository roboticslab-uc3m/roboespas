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
        lbr
        qWaypoints
        tWaypoints
        npoints
    end
    
    methods
        
        function obj = IiwaTrajectory(varargin) %lbr, name, tWaypoints, qWaypoints, t)
            if (length(varargin)==5)
                obj=obj.wayPointsConstructor(varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5});
            elseif (length(varargin)==3)
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
                    obj.lbr=varargin{1}.lbr;
                    obj.qWaypoints=varargin{1}.qWaypoints;
                    obj.tWaypoints=varargin{1}.tWaypoints;
                    obj.name=varargin{1}.name;
                    obj.npoints=varargin{1}.npoints;
                end
            end
            obj=obj.CompleteCartesian();
        end
        function obj = CompleteCartesian(obj)
            for i=1:size(obj.q,1)
                obj.x(i,:)=IiwaScrewTheory.ForwardKinematics(obj.q(i,:));
            end
        end
        function obj = CompleteEffort(obj, modeID)
            if (strcmp(modeID,'st')==1)
               	for i = 1:size(obj.q,1)
                    tic
                    obj.effort(i,:) = IiwaScrewTheory.InverseDynamics(obj.q(i,:), obj.qdot(i,:), obj.qdotdot(i,:));
                    asdf = [obj.q(i,:); obj.qdot(i,:); obj.qdotdot(i,:); obj.effort(i,:)]
                    toc
                end
            elseif (strcmp(modeID,'matlab')==1)
                for i = 1:size(obj.q,1)
                    obj.effort(i,:) = inverseDynamics(obj.lbr, obj.q(i,:), obj.qdot(i,:), obj.qdotdot(i,:));
                    asdf = [obj.q(i,:); obj.qdot(i,:); obj.qdotdot(i,:); obj.effort(i,:)]
                end
            end
        end
        function obj = CompleteJoint(obj, qini)
            obj.q(1,:)=qini;
            for i=1:size(obj.x,1)-1
                obj.xdot(i,:) = IiwaScrewTheory.screwA2B_A(obj.x(i,:), obj.x(i+1,:))/(obj.t(i+1)-obj.t(i));
                xdot_S = IiwaScrewTheory.tfscrew_A2S(obj.xdot(i,:), obj.x(i,:));
                obj.qdot(i,:) = IiwaScrewTheory.IDK_point(obj.q(i,:), xdot_S);
                obj.q(i+1,:) = obj.q(i,:) + obj.qdot(i,:)*(obj.t(i+1)-obj.t(i));
            end
            obj.xdot(size(obj.x,1),:)=zeros(6,1);
            obj.qdot(size(obj.x,1),:)=zeros(size(IiwaRobot.Twist,2),1);
            obj.CompleteCartesian();
        end
        function obj = wayPointsConstructor(obj, lbr, name, tWaypoints, qWaypoints, t)
            %Creates a trajectory that passes through some joint positions
            %at certain timestamps
            obj.tWaypoints = tWaypoints;
            obj.qWaypoints = qWaypoints;
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
            obj.lbr=lbr;
            obj.npoints = length(obj.t);
        end
    end
    methods (Static)
        %% Funciones Nacho
        function [t_trayectoria,q_trayectoria] = redireccionarInicioTrayectoria(tWaypoints,qWaypoints)

            % homeRobotPosition = joint positions [0,0,0,0,0,0,0]
            %
            % Comprobation to know if the trayectory start in homeRobotPosition or
            % not

            pto_1 = qWaypoints(1,:);
            pto_2 = qWaypoints(end,:);

            % Establecemos tiempo y duración de la trayectoria

                % La duración de la trayectoria se establece teniendo en cuenta los
                % limites de velocidad articular de cada articulación

            % q_trayectoria = [puntosRadianes(pto_1,:);
            %                  puntosRadianes(pto_2,:)];

            % distancia_q1 = abs(abs(puntosRadianes(pto_1,1))-abs(puntosRadianes(pto_2,1)));
            % distancia_q2 = abs(abs(puntosRadianes(pto_1,2))-abs(puntosRadianes(pto_2,2)));

            for i = 1:7

                distancia_q0 = abs(pto_1);

                if pto_1(i) > 0 && pto_2(i) > 0 || pto_1(i) < 0 && pto_2(i) < 0 % Si tienen mismo signo pto_1 y pto_2
                    if pto_1(i) < pto_2(i)
                       distancia_q2(i) = abs(abs(pto_2(i))-abs(pto_1(i)));
                    else
                       distancia_q2(i) = abs(abs(pto_1(i))-abs(pto_2(i)));
                    end
                else % Si tiene distinto signo pto_1 y pto_2
                    distancia_q2(i) = abs(abs(pto_1(i))+abs(pto_2(i)));
                end

            end

            % - Trayectoria de recolocación - (de homeposition del robot a inicio de
            % trayectoria)
            tiempo1 = 0;

            if pto_1(1) ~= 0 | pto_1(2) ~= 0 | pto_1(3) ~= 0 | pto_1(4) ~= 0 | pto_1(5) ~= 0 | pto_1(6) ~= 0 | pto_1(7) ~= 0
                % CONTINUAMOS CON EL PROGRAMA IJL_DataAcquisition.m
                tiempo1 = 1;
                while distancia_q0(1) > 1.31 | distancia_q0(2) > 1.31 | distancia_q0(3) > 1.31 | distancia_q0(4) > 1.31 | distancia_q0(5) > 1.31 | distancia_q0(6) > 1.31 | distancia_q0(7) > 1.31
                    tiempo1 = tiempo1+1; 
                    distancia_q0 = distancia_q0 - 1.31;
                end
            end

            % Si la trayectoria no comienza en el homePosition se añade la trayectoria
            % desde el homeposition hasta el punto inicial de la trayectoria. Si
            % comienza en el homeposition no se añade.

            if pto_1(1) ~= 0 | pto_1(2) ~= 0 | pto_1(3) ~= 0 | pto_1(4) ~= 0 | pto_1(5) ~= 0 | pto_1(6) ~= 0 | pto_1(7) ~= 0
                q_trayectoria(1,:) = [0 0 0 0 0 0 0];
                t_trayectoria = [0];
                    for i=1:size(tWaypoints,2)
                        tiempo2 = tiempo1+tWaypoints(i);
                        t_trayectoria(i+1) = tiempo2;

                        q_trayectoria(i+1,:) = qWaypoints(i,:);
                    end

                %t_trayectoria = [0,tiempo1,tiempo2];

            else
                q_trayectoria = qWaypoints;
                t_trayectoria = tWaypoints;
            end

        end
    end
end

