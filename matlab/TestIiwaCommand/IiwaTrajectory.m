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
    end
    
    methods
        
        function obj = IiwaTrajectory(varargin) %lbr, name, tWaypoints, qWaypoints, t)
            if (length(varargin)==5)
                obj=obj.wayPointsConstructor(varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5});
            elseif (length(varargin)==2)
                obj=IiwaMsgTransformer.toIiwaTrajectory(varargin{1}, varargin{2});
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
                end
            end
            obj=obj.CompleteCartesian();
        end
        function obj = CompleteCartesian(obj)
            for i=1:size(obj.q,1)
                obj.x(i,:)=ScrewTheory.ForwardKinematics(obj.q(i,:));
            end
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
           
            for i = 1:size(obj.q,1)
                obj.effort(i,:) = inverseDynamics(lbr, obj.q(i,:), obj.qdot(i,:), obj.qdotdot(i,:));
            end
            obj.lbr=lbr;
        end
    end
    methods (Static)
        %% Funciones Ana
        function traj = TrapezoidalVelocityTrajectoryParameterized(tacc, tflat, a, time_step, name)
            %Returns a 1 coordinate trajectory with v0=0 and vend=0, given 
            %a certain time for accelerating/deccelerating, a time in which
            %the velocity maintains constant, the acceleration and the 
            %time step, as well as the name for the given trajectory
            traj=IiwaTrajectory(name);
            if (isinf(tacc) || isinf(tflat))
                ME=MException('IiwaTrajectory:infiniteTimes', 'Acceleration and flat times must be non infinite');
                throw(ME);
                return;
            end
            ttotal = tacc*2 + tflat;
            %Build parameterized trajectory
            ids_acc = 2:round(tacc/time_step)+1;
            ids_flat = round(tacc/time_step)+1:round((tacc+tflat)/time_step)+1;
            ids_dec = round((tacc+tflat)/time_step)+1:round(ttotal/time_step)+1;

            traj.xdotdot(ids_acc,:) = a;
            traj.xdotdot(ids_flat,:) = 0;
            traj.xdotdot(ids_dec(1:end-1),:) = -a;
            traj.xdotdot(ids_dec(end),:) = 0;
            traj.t = (0:time_step:ttotal)';
            traj.x = zeros(size(traj.t,1), 1);
            traj.xdot = zeros(size(traj.t,1), 1);
            traj.x(1) = 0;
            traj.xdot(1) = 0;
            for i=ids_acc(2:end)
                traj.x(i) = 0.5 * a * traj.t(i)*traj.t(i);
                traj.xdot(i) = a*traj.t(i);
            end
            for i=ids_flat(2:end)
               t_=traj.t(i)-traj.t(ids_flat(1));
               traj.x(i) = traj.x(ids_flat(1))+traj.xdot(ids_flat(1))*t_;
               traj.xdot(i)=traj.xdot(ids_flat(1));
            end
            for i=ids_dec(2:end)
                t_=traj.t(i)-traj.t(ids_dec(1));
                traj.x(i) = traj.x(ids_dec(1)) + traj.xdot(ids_dec(1))*t_-0.5*a*t_*t_;
                traj.xdot(i) = traj.xdot(ids_dec(1)) -a*t_;
            end
        end
        function traj = Deparametrize(traj_pos, traj_ori, axis_tras, axis_rot, xini, name)
            traj = IiwaTrajectory(name);
            traj.t = traj_pos.t;
            traj.xdot = [traj_pos.xdot*axis_tras, traj_ori.xdot*axis_rot];
            traj.xdotdot = [traj_pos.xdotdot*axis_tras, traj_ori.xdotdot*axis_rot];
            for i=1:size(traj.t,1)
                traj.x(i,:) = ScrewTheory.tfframe_A(xini, [traj_pos.x(i)*axis_tras, traj_ori.x(i)*axis_rot]);
            end
        end
        function traj = TrapezoidalVelocityTrajectory(qini, xgoal, ttotal, control_step_size, name)
            xini= ScrewTheory.ForwardKinematics(qini);
            xinc_A = ScrewTheory.screwA2B_A(xini, xgoal);

            axis_rot=xinc_A(4:6)/norm(xinc_A(4:6))
            angle=norm(xinc_A(4:6))
            
            axis_tras=xinc_A(1:3)/norm(xinc_A(1:3))
            dist=norm(xinc_A(1:3))
            
            
            [a_pos, tacc, tflat] = ScrewTheory.GetTrapezoidalTrajectoryTimeParameters(ttotal, dist, IiwaRobot.CartAccMax, control_step_size);
            a_ori = ScrewTheory.GetTrapezoidalTrajectoryAcceleration(angle, tacc, tflat);
            
            %Build trajectories with the given acceleration, tacc, tflat
            %and control_step_size
            traj_para_pos = IiwaTrajectory.TrapezoidalVelocityTrajectoryParameterized(tacc, tflat, a_pos, control_step_size, 'parametrized_pos');
            traj_para_ori = IiwaTrajectory.TrapezoidalVelocityTrajectoryParameterized(tacc, tflat, a_ori, control_step_size, 'parametrized_ori');
            
            %Deparametrize
            traj = IiwaTrajectory.Deparametrize(traj_para_pos, traj_para_ori, axis_tras, axis_rot, xini, name);
        end
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
        function [t,q] = ExampleTrajectoryIJL(numero)
            % Independent Joint Learning
            % tWaypoints: Time vector
            % qWaypoints: Trayectory vector
            %
            %
            % References take from my own file: IJL_DataAdquisition
            switch numero 
                case 1      
                    disp('Trayectoria 1');
                    A = pi/2;
                    T = 14;
                    n = 1;
                    t = 0:0.1:T;

                    % Creamos la trayectoria para 7 articulaciones

                    for i=1:7
                       q(:,i) = A*cos((n*t)/T); 
                    end

                    %figure('Name','Joint'); plot(t,q);

                    % Sampling rate of 100 Hz = 0.01 s using the Fast Research Interface (FRI)
                    %
                    % joints’ torques were filtered through a 4th order zero-
                    % phase digital Butterworth filter with a cuttoff frequency of
                    % 1 Hz

                    % Class Gazebo>SensorTorqueForce: http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1sensors_1_1ForceTorqueSensor.html
                    % sensor torque: https://answers.gazebosim.org//question/18715/does-anyone-have-a-working-example-or-a-tutorial-for-a-force_torque-plugin/

                case 2

                    disp('Trayectoria 2');

                    A = [1.4 1.7 1.6 1 1.2 1.4 1.6];
                    T = 20;
                    n = [3 1 2 1 1 1 2];
                    t = 0:0.1:T;

                    % Creamos la trayectoria para 7 articulaciones

                    for i=1:7
                       q(:,i) = A(i)*cos((n(i)*t)/T);

                       % Representamos las trayectorias
                       JointName = strcat('Joint',int2str(i));
                       %figure('Name',JointName); plot(t,q(:,i));
                    end

                case 3

                    disp('Trayectoria 3');

                    A = [1.4 1.4 1.8 1.4 1.6 1.7 1.2];
                    T = 20;
                    n = [1 3 2 1 2 3 2];
                    t = 0:0.1:T;

                    % Creamos la trayectoria para 7 articulaciones

                    for i=1:7
                       q(:,i) = A(i)*cos((n(i)*t)/T);

                       % Representamos las trayectorias
                       JointName = strcat('Joint',int2str(i));
                       %figure('Name',JointName); plot(t,q(:,i));
                    end

                case 4

                    disp('Trayectoria 4');

                    A = [2 1.1 1.8 1.5 1.7 2 1.6];
                    T = 60;
                    n = [2 1 2 1 3 3 2];
                    t = 0:0.1:T;

                    % Creamos la trayectoria para 7 articulaciones

                    for i=1:7
                       q(:,i) = A(i)*cos((n(i)*t)/T);

                       % Representamos las trayectorias
                       JointName = strcat('Joint',int2str(i));
                       %figure('Name',JointName); plot(t,q(:,i));
                    end

                case 5

                    disp('Trayectoria 5');

                    A = [1.5 1.7 1.4 1.1 1.9 1.2 1.6];
                    T = 20;
                    n = [2 1 2 2 2 1 3];
                    t = 0:0.1:T;

                    % Creamos la trayectoria para 7 articulaciones

                    for i=1:7
                       q(:,i) = A(i)*cos((n(i)*t)/T);

                       % Representamos las trayectorias
                       JointName = strcat('Joint',int2str(i));
                       %figure('Name',JointName); plot(t,q(:,i));
                    end

                 case 6

                    disp('Trayectoria 6');

                    A = [1.7 1.1 1.4 1.9 1.4 1.7 1.2];
                    T = 20;
                    n = [1 2 3 3 2 1 3];
                    t = 0:0.1:T;

                    % Creamos la trayectoria para 7 articulaciones

                    for i=1:7
                       q(:,i) = A(i)*cos((n(i)*t)/T);

                       % Representamos las trayectorias
                       JointName = strcat('Joint',int2str(i));
                       %figure('Name',JointName); plot(t,q(:,i));
                    end

                 case 7

                    disp('Trayectoria 7');

                    A = [1.1 1.4 1.6 1.2 0.9 1.9 1.3];
                    T = 30;
                    n = [1 2 2 1 3 3 3];
                    t = 0:0.1:T;

                    % Creamos la trayectoria para 7 articulaciones

                    for i=1:7
                       q(:,i) = A(i)*cos((n(i)*t)/T);

                       % Representamos las trayectorias
                       JointName = strcat('Joint',int2str(i));
                       %figure('Name',JointName); plot(t,q(:,i));
                    end

                 case 8

                    disp('Trayectoria 8');

                    A = [0.9 1.4 1.7 1.5 1.9 1.2 1.6]
                    T = 30;
                    n = [3 3 2 2 1 1 2];
                    t = 0:0.1:T;

                    % Creamos la trayectoria para 7 articulaciones

                    for i=1:7
                       q(:,i) = A(i)*cos((n(i)*t)/T);

                       % Representamos las trayectorias
                       JointName = strcat('Joint',int2str(i));
                       %figure('Name',JointName); plot(t,q(:,i));
                    end

                 case 9

                    disp('Trayectoria 9');

                    A = [1.2 1.3 1.1 1.6 1.3 1.5 1.5];
                    T = 20;
                    n = [3 2 2 3 1 1 2];
                    t = 0:0.1:T;

                    % Creamos la trayectoria para 7 articulaciones

                    for i=1:7
                       q(:,i) = A(i)*cos((n(i)*t)/T);

                       % Representamos las trayectorias
                       JointName = strcat('Joint',int2str(i));
                       %figure('Name',JointName); plot(t,q(:,i));
                    end        
            end
        end
    end
end

