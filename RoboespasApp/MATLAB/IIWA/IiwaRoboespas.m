classdef IiwaRoboespas < handle
    %IiwaRoboespas permite interactuar con el IIWA desde la aplicación
    %RoboespasApp.mlapp. Utiliza las clases:
    % * IiwaCommandFRI
    % * IiwaCommandStack
    % * IiwaPlotter
    % * IiwaTrajectory
    
    %% CONSTANTES PRIVADAS
    properties(Constant, Access=private)
        %Outside: Permite que las gráficas se muestren en una figura
        %externa en lugar de en la aplicación RoboespasApp
        Outside = 0;
        
        %Posiciones iniciales para la captura de trayectorias según el
        %brazo seleccionado
        ReferencePositionRight = [-1.7910, 1.1126, -2.1927, 1.8522, -1.2179, -0.8831, -0.2125];
        ReferencePositionLeft = [-2.7635, 1.1307, -2.1927, 1.8343, -1.2077, -0.8865, -1.2047];

        %Colores escogidos para las gráfica
        ColorCommanded=[0.93, 0.69, 0.13];
        ColorTrials=[0.85, 0.33, 0.10];
        
        %Parámetros para salvar las trayectorias grabadas y ajustadas
        TrajectoriesFolder = [cell2mat(extractBetween(fileparts(which(mfilename)), 1, strfind(fileparts(which(mfilename)), 'RoboespasApp')+strlength('RoboespasApp'))), 'Results/IIWATrajectories/CurrentTrajectories'];
        AdjustedIdentifier='adj';
        InputIdentifier='cap';
        
        %Tiempo de remuestreo escogido
        CommandedSampleTime = 0.005 %seconds
        
        %Parámetros del spline escogidos
        nSegCommand = 50;
        SmoothingCommand = 1e-5;
    end
    %% VARIABLES LEIBLES NO EDITABLES (algunas son editables a través de métodos)
    properties(GetAccess=public, SetAccess=private)
        %Variable que muestra si está ROS conectado, es decir, si se está
        %interactuando con el robot
        ROSConnected = 0
        %CommandMode es 'Stack' o 'FRI', según el nombre del nodo que se
        %encuentre en el momento de conectar Matlab a ROS
        CommandMode = 'Stack' 
        %IiwaCommand es un objeto de tipo IiwaCommandStack o IiwaCommandFRI
        %según el modo de comando escogido
        IiwaCommand 

        %Velocidad angular de la trayectoria grabada
        circle_angvel
        %Velocidad angular de la trayectoria comandada
        circle_angvel_commanded
        
        %Nombre escogido para la trayectoria actual
        NameData
        %Parámetro utilizado para dar nombre a las trayectorias grabadas
        NameMovement = 'flexext'
        %Nombre de la trayectoria de entrada, será NameData+InputIdentifier
        NameDataInput
        %Nombre de la trayectoria de salida, será
        %NameData+AdjustedIdentifier
        NameDataAdjusted
        %Fecha de captura de la trayectoria
        CapturedDate
       
        %Pausa entre flexión y extensión, ajustable
        CommandedMiddlePause = 2;
        
        %TRAYECTORIAS: De tipo IiwaTrajectory, contienen diferentes
        %trayectorias utilizadas en diferentes partes
        %Trayectoria de entrada
        traj_input
        %Trayectoria grabada resampleada
        traj_resampled
        %Trayectoria grabada tras quitarle las pausas inicial/final
        traj_input_no_pauses
        %Trayectoria de flexión ajustada a un arco de circulo
        traj_adjusted
        %Trayectoria grabada reescalada a la velocidad escogida
        traj_input_show
        %Trayectoria de flexión ajustada a círculo y a la velocidad deseada
        traj_velocity_circle
        %Trayectoria de flexión ajustada tras el spline
        traj_spline
        %Trayectoria de flexoextensión
        traj_flexoext
        %Trayectoria comandada
        traj_command
        %Trayectoria leída durante la ejecución de la referencia
        traj_reference
        %Trayectorias leídas durante las repeticiones ejecutadas
        trajs_trial={};
        %Trayectorias que contienen valores min, med y max de todas
        %las ejecuciones realizadas
        traj_min
        traj_mean
        traj_max
        %Trayectorias que contienen valores min, med y max del
        %error entre cada ejecución realizada y la trayectoria comandada
        traj_min_error
        traj_mean_error
        traj_max_error
        %Trayectorias que contienen valores min, med y max del
        %error entre cada ejecución realizada y la media de las ejecuciones
        traj_min_error_rep
        traj_mean_error_rep
        traj_max_error_rep 
    end
    methods
%% CONSTRUCTOR
        function obj = IiwaRoboespas()
            %Constructor de IiwaRoboespas.
            %ROS se supone desconectado por el momento, se inicializa la 
            %clase para mostrar gráficas IiwaPlotter
            obj.ROSConnected=0;
            IiwaPlotter.Initialize();
        end
%% FUNCIONES PÚBLICAS
        %FUNCIONES GENERALES
        function CommandMode = ConnectROS(obj)
            %Función que inicializar la comunicación con ROS, utilizando la
            %función externa init_ros. Se ha mantenido como externa para
            %poder utilizarla aunque no se esté utilizando RoboespasApp.
            init_ros;
            obj.ROSConnected=1;
            %Limpiar las clases de comunicación con ROS
            clear IiwaCommandStack;
            clear IiwaCommandFRI;
            clear IiwaMsgTransformer;
           
            %Comprobar, de entre los nodos disponibles en la red, si está
            %iiwa_command_stack o iiwa_command_fri, para devolver el tipo
            %de interfaz de comunicación que se utilizará de forma
            %informativa.
            if (contains(cell2mat(rosnode('list')), 'stack'))
                obj.CommandMode = 'Stack';
                obj.IiwaCommand = IiwaCommandStack;
            elseif(contains(cell2mat(rosnode('list')), 'fri'))
                obj.CommandMode = 'FRI';
                obj.IiwaCommand = IiwaCommandFRI;
            else
                ME=MException('IiwaRoboespas:IiwaCommandMode', 'IiwaRoboespas.CommandMode must be "FRI" or "Stack"');
                throw(ME);
            end
            %Subir la velocidad, aceleración y jerk al maximo. 
            obj.IiwaCommand.SetVelocity(1);
            obj.IiwaCommand.SetAcceleration(1);
            obj.IiwaCommand.SetJerk(1);
            %Devolver 'FRI' o 'Stack'
            CommandMode = obj.CommandMode;
        end
        % FUNCIONES DE MOVIMIENTO DEL ROBOT
        function SendReferencePositionLeft(obj)
            %Función que mueve el IIWA hasta la posicion de referencia
            %izquierda
            if (obj.ROSConnected)
                %Bajar la velocidad
                obj.IiwaCommand.SetVelocity(0.1);
                %Mover el robot utilizando la interfaz que corresponda
                %(IiwaCommand es IiwaCommandFRI o IiwaCommandStack según el
                %caso)
            	[~, ~] = obj.IiwaCommand.MoveJ(obj.ReferencePositionLeft);
                %No se utilizan las salidas pero podría obtenerse la
                %trayectoria comandada y ejecutada si fuera necesario
            end
        end
        function SendReferencePositionRight(obj)
            %Función que mueve el IIWA hasta la posicion de referencia
            %derecha
            if (obj.ROSConnected)
                %Bajar la velocidad
                obj.IiwaCommand.SetVelocity(0.1);
                %Mover el robot utilizando la interfaz que corresponda
                %(IiwaCommand es IiwaCommandFRI o IiwaCommandStack según el
                %caso)
                [~, ~] = obj.IiwaCommand.MoveJ(obj.ReferencePositionRight);
                %No se utilizan las salidas pero podría obtenerse la
                %trayectoria comandada y ejecutada si fuera necesario
            end
        end
        function SendInitialPosition(obj)
            %Función que mueve el IIWA hasta la posicion inicial de la
            %trayectoria a comandar (traj_flexoext).
           	if (obj.ROSConnected)
                %Bajar la velocidad
                obj.IiwaCommand.SetVelocity(0.1);
                if (~isempty(obj.traj_flexoext))
                    %Mover el robot utilizando la interfaz que corresponda
                    %(IiwaCommand es IiwaCommandFRI o IiwaCommandStack según el
                    %caso)
                    [~, ~] = obj.IiwaCommand.MoveJ(obj.traj_flexoext.q(1,:));
                    %No se utilizan las salidas pero podría obtenerse la
                    %trayectoria comandada y ejecutada si fuera necesario
                else
                    %Si la trayectoria estaba vacía, lanzar una excepción
                    ME=MException('IiwaRoboespas:EmptyTrajFlexoext', 'traj_flexoext vacio');
                    throw(ME);
                end
            end
        end
        function [IDPatient, Arm] = SendReferenceTrajectory(obj)
            [IDPatient, Arm] = obj.SetNameData(obj.NameData);
            if (obj.ROSConnected)
                obj.IiwaCommand.SetVelocity(1);
                [traj_command, obj.traj_reference] = obj.IiwaCommand.SendTraj(obj.traj_flexoext);
            else
                pause(5);
            end
        end
        function [IDPatient, Arm] = SendTrialTrajectory(obj, i_trial)
            [IDPatient, Arm] = obj.SetNameData(obj.NameData);
            if (obj.ROSConnected)
                obj.IiwaCommand.SetVelocity(1);
                [obj.traj_command, obj.trajs_trial{i_trial}] = obj.IiwaCommand.SendTraj(obj.traj_flexoext);
            else
                pause(5);
            end
        end
        % CAPTURE FUNCTIONS
        function CaptureStart(obj)
            if (obj.ROSConnected)
                obj.IiwaCommand.StartCapture();
                obj.IiwaCommand.GravityCompensationMode('cartesian', 'YZC');
            end
        end
        % CONFIGURATION FUNCTIONS
        function FreeCartesianCoordinate(obj, coord) %X, Y, Z, A, B, C
            if (obj.ROSConnected)
                obj.IiwaCommand.GravityCompensationMode('cartesian', coord);
            end
        end
        function BlockRobot(obj)
            if (obj.ROSConnected)
                obj.IiwaCommand.GravityCompensationMode('stop');
            end
        end
        % DATA INPUT FUNCTIONS
        function InputList = DataInputList(obj)
            listing = dir(obj.TrajectoriesFolder);
            InputList = {};
            n=1;
            for i=1:length(listing)
                if (strcmp(listing(i).name, '.')~=1 && strcmp(listing(i).name, '..')~=1)
                    if(~isempty(strfind(listing(i).name, obj.InputIdentifier)))
                        InputList{n} = erase(listing(i).name, '.mat'); %#ok<AGROW>
                        n=n+1;
                    end
                end
            end
        end
        function [IDPatient, Arm] = LoadInput(obj, name_input) 
            [IDPatient, Arm] = obj.SetNameData(name_input(1:strfind(name_input, obj.InputIdentifier)-1));
            %Load data input
            obj.traj_input = obj.LoadTrajectory(obj.NameDataInput);
            obj.traj_input_no_pauses = obj.traj_input.DeleteInitialEndPauses();
        end
        function SaveDataInput(obj,  id_patient, arm)
            obj.CapturedDate=datestr(now, 'yymmdd_HHMMSS');
            obj.BuildNameData(id_patient, arm); %This automatically changes also obj.NameDataInput
            if (obj.ROSConnected)
                obj.traj_input = obj.IiwaCommand.StopCapture();  
                obj.traj_input.name = 'input';
                obj.SaveTrajectory(obj.traj_input, obj.NameDataInput);
            else
                obj.DefaultCapTrajectoryList=[obj.DefaultCapTrajectoryList, {obj.NameDataInput}];
            end
        end
        function SaveTrajectory(obj, traj, folder)
            [ ~, ~ ] = mkdir(obj.TrajectoriesFolder);
            angvel = obj.circle_angvel;
            save([obj.TrajectoriesFolder, '/', folder, '.mat'], 'traj', 'angvel');
        end
        function traj = LoadTrajectory(obj, name)
            traj = load([obj.TrajectoriesFolder, '/', name, '.mat']);
            if (~isempty(traj.angvel))
                obj.circle_angvel = traj.angvel;
            end
            traj = traj.traj;
        end
        function DeleteInputTrajectory(obj)
            filepath = [obj.TrajectoriesFolder, '/', obj.NameDataInput, '.mat'];
            delete(filepath);
        end
        function DeleteAdjustedTrajectory(obj)
            filepath = [obj.TrajectoriesFolder, '/', obj.NameDataAdjusted, '.mat'];
            delete(filepath);
        end
        % DATA ADJUSTED/COMMANDED FUNCTIONS
        function AdjustedList = DataAdjustedList(obj)
            listing = dir(obj.TrajectoriesFolder);
            AdjustedList = {};
            n=1;
            for i=1:length(listing)
                if (strcmp(listing(i).name, '.')~=1 && strcmp(listing(i).name, '..')~=1)
                    if(~isempty(strfind(listing(i).name, obj.AdjustedIdentifier)))
                        AdjustedList{n} = erase(listing(i).name, '.mat'); %#ok<AGROW>
                        n=n+1;
                    end
                end
            end
        end
        function [IDPatient, Arm] = LoadAdjusted(obj, name_commanded) 
            [IDPatient, Arm] = obj.SetNameData(name_commanded(1:strfind(name_commanded, obj.AdjustedIdentifier)-1));
            %Load data adjusted
            obj.traj_adjusted = obj.LoadTrajectory(obj.NameDataAdjusted);
        end
        function [IDPatient, Arm, e_rms] = BuildDataAdjusted(obj, name_input, display)
            [IDPatient, Arm] = obj.SetNameData(name_input(1:strfind(name_input, obj.InputIdentifier)-1));
            if (obj.Outside==1)
                display = figure;
            end
            obj.traj_resampled = obj.traj_input.ChangeSampleTime(obj.CommandedSampleTime);
            obj.traj_input_no_pauses = obj.traj_resampled.DeleteInitialEndPauses();
            [obj.traj_adjusted, C, R, arc, obj.circle_angvel, e_rms] = obj.traj_input_no_pauses.FitToCircle('X', display);
            obj.traj_adjusted.name = 'adjusted';
            obj.SaveTrajectory(obj.traj_adjusted, obj.NameDataAdjusted);
        end
        
        function ChangeVelocity(obj, w_out)
            obj.circle_angvel_commanded=w_out;
            if (~isempty(obj.traj_adjusted))
                obj.traj_velocity_circle = obj.traj_adjusted.ChangeVelocity(w_out/obj.circle_angvel);
                obj.traj_spline = obj.traj_velocity_circle.BoundedSpline(obj.SmoothingCommand, obj.nSegCommand);
                traj_spline_pause = obj.traj_spline.AddPause(obj.CommandedMiddlePause);
                obj.traj_flexoext = traj_spline_pause.MergeAfterwards(obj.traj_spline.MirrorTrajectory());
                obj.traj_flexoext.name = 'mirrored';
                obj.traj_flexoext = obj.traj_flexoext.FixCartesianCoordinates('XBC'); %For commanding cartesian velocities
                if (~isempty(obj.traj_input))
                    obj.traj_input_show = obj.traj_input_no_pauses.ChangeVelocity(w_out/obj.circle_angvel);
                else
                    ME=MException('IiwaRoboespas:EmptyTrajInput', 'traj_input esta vacio');
                    throw(ME);
                end
            else
                ME=MException('IiwaRoboespas:EmptyTrajAdjusted', 'traj_adjusted esta vacio');
                throw(ME);
            end
        end
        % DATA OUTPUT FUNCTIONS
        function Trajectory = GetStruct(obj, i_trial)
            %%ADded as it is
            if (obj.ROSConnected)
                ppsec=3;
                v=obj.circle_angvel_commanded;
                Trajectory.CapturedDate = obj.CapturedDate;
                Trajectory.Reference.IiwaTrajectory = obj.traj_reference;
                Trajectory.Reference.Timestamps = obj.traj_reference.t;
                Trajectory.Reference.JointTrajectory = obj.traj_reference.q;
                Trajectory.Reference.CartesianTrajectory.Position = obj.traj_reference.x(:,1:3);
                Trajectory.Reference.CartesianTrajectory.Orientation = obj.traj_reference.x(:,4:6);
                Trajectory.Reference.JointVelocities = obj.traj_reference.qdot;
                Trajectory.Reference.CartesianVelocities.Position = obj.traj_reference.xdot(:,1:3);
                Trajectory.Reference.CartesianVelocities.Orientation = obj.traj_reference.xdot(:,4:6);
                Trajectory.Reference.JointAccelerations = obj.traj_reference.qdotdot;
                Trajectory.Reference.JointTorques.Torques = obj.traj_reference.effort;
                Trajectory.Trial.IiwaTrajectory = obj.trajs_trial{i_trial};
                Trajectory.Trial.Timestamps = obj.trajs_trial{i_trial}.t;
                Trajectory.Trial.JointTrajectory = obj.trajs_trial{i_trial}.q;
                Trajectory.Trial.CartesianTrajectory.Position = obj.trajs_trial{i_trial}.x(:,1:3);
                Trajectory.Trial.CartesianTrajectory.Orientation = obj.trajs_trial{i_trial}.x(:,4:6);
                Trajectory.Trial.JointVelocities = obj.trajs_trial{i_trial}.qdot;
                Trajectory.Trial.CartesianVelocities.Position = obj.trajs_trial{i_trial}.xdot(:,1:3);
                Trajectory.Trial.CartesianVelocities.Orientation = obj.trajs_trial{i_trial}.xdot(:,4:6);
                Trajectory.Trial.JointAccelerations = obj.trajs_trial{i_trial}.qdotdot;
                Trajectory.Trial.JointTorques.Torques = obj.trajs_trial{i_trial}.effort;
                Trajectory.Commanded.IiwaTrajectory = obj.traj_command;
                Trajectory.Commanded.Timestamps = obj.traj_command.t;
                Trajectory.Commanded.JointTrajectory = obj.traj_command.q;
                Trajectory.Commanded.CartesianTrajectory.Position = obj.traj_command.x(:,1:3);
                Trajectory.Commanded.CartesianTrajectory.Orientation = obj.traj_command.x(:,4:6);
                Trajectory.Commanded.JointVelocities = obj.traj_command.qdot;
                Trajectory.Commanded.CartesianVelocities.Position = obj.traj_command.xdot(:,1:3);
                Trajectory.Commanded.CartesianVelocities.Orientation = obj.traj_command.xdot(:,4:6);
                Trajectory.Commanded.JointAccelerations = obj.traj_command.qdotdot;
                Trajectory.Commanded.JointTorques.Torques = obj.traj_command.effort;
            else
                Trajectory=[];
            end
        end
        % SET PRIVATE PROPERTIES FUNCTIONS -> Allow to change some
        % properties but just in specific ways
        function SetMovement(obj, name)
            obj.NameMovement=name;
        end
        function [IDPatient, Arm] = SetNameData(obj, name_data)
            obj.NameData=name_data;
            obj.NameDataAdjusted=[obj.NameData, obj.AdjustedIdentifier];
            obj.NameDataInput=[obj.NameData, obj.InputIdentifier];
            numbersinnamedata=regexp(name_data, '\d*', 'match');
            IDPatient=str2double(numbersinnamedata{1});
            obj.CapturedDate=[numbersinnamedata{2}, '_', numbersinnamedata{3}];
            if (contains(name_data, '_I_'))
                Arm='I';
            elseif (contains(name_data, '_D_'))
                Arm='D';
            end
        end
        function SetSmoothingParameters(obj, smoothing)
            obj.SmoothingCommand = smoothing;
        end
        function SetCommandedPPSegments(obj, segments)
            obj.nSegCommand = segments;
        end
        function SetCommandedMiddlePause(obj, pause)
            obj.CommandedMiddlePause = pause;
        end
        function BuildNameData(obj, id_patient, arm)
            obj.NameData=strcat(obj.NameMovement, '_P', num2str(id_patient, '%03.f'), '_', arm, '_', obj.CapturedDate, '_');
            obj.NameDataAdjusted=[obj.NameData, obj.AdjustedIdentifier];
            obj.NameDataInput=[obj.NameData, obj.InputIdentifier];
        end
        % PLOT FUNCTIONS
        function GetMinMeanMax(obj)
            [obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error, obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep] = IiwaTrajectory.MinMeanMaxTrajectories(obj.trajs_trial, obj.traj_command);
        end
        function ClearTrials(obj)
            obj.trajs_trial={};
            obj.traj_min=[];
            obj.traj_mean=[];
            obj.traj_max=[];
            obj.traj_min_error=[];
            obj.traj_mean_error=[];
            obj.traj_max_error=[];
            obj.traj_min_error_rep=[];
            obj.traj_mean_error_rep=[];
            obj.traj_max_error_rep=[];
        end
        % Plot trajectories
        function PlotCircleAdjustment(obj, display)
            if (obj.Outside==1)
                display = figure;
            end
            sgtitle(display, 'posicion cartesiana tridimensional capturada y ajustada');
            IiwaPlotter.cartesian_positions3d({obj.traj_velocity_circle, obj.traj_spline, obj.traj_flexoext}, ['b', 'r', 'g', 'm'], display);     
        end
        function PlotJointPositionTrajectories(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            if (obj.ROSConnected)
                if (i_trial~=0)
                    sgtitle(display, ['Posiciones articulares de la trayectoria comandada y ejecutada', num2str(i_trial)]);
                    IiwaPlotter.joint_positions({obj.traj_command, obj.trajs_trial{i_trial}}, {obj.ColorCommanded, obj.ColorTrials}, display)
                else
                    sgtitle(display, 'Posiciones articulares de la trayectoria comandada y ejecutada min, med y max');
                    IiwaPlotter.fill_joint_position(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
                    IiwaPlotter.joint_positions({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials, obj.ColorTrials, obj.ColorCommanded, '--'}, display)
                    display.Children(1).String={'output', 'min', 'mean', 'max', 'commanded'};
                end
            end
        end
        function PlotJointVelocityTrajectories(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            if (obj.ROSConnected)
                if (i_trial~=0)
                    sgtitle(display, ['Velocidades articulares de la trayectoria comandada y ejecutada', num2str(i_trial)]);
                    IiwaPlotter.joint_velocities({obj.traj_command, obj.trajs_trial{i_trial}}, {obj.ColorCommanded, obj.ColorTrials, '.'}, display)
                else
                    sgtitle(display, 'Velocidades articulares de la trayectoria comandada y ejecutada min, med y max');
                    IiwaPlotter.fill_joint_velocity(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
                    IiwaPlotter.joint_velocities({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials,obj.ColorTrials, obj.ColorCommanded, '--'}, display)
                    display.Children(1).String={'output', 'min', 'mean', 'max', 'commanded'};
                end
            end
        end
        function PlotJointAccelerationTrajectories(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            if (obj.ROSConnected)
                if (i_trial~=0)
                    sgtitle(display, ['Aceleraciones articulares de la trayectoria comandada y ejecutada', num2str(i_trial)]);
                    IiwaPlotter.joint_accelerations({obj.traj_command, obj.trajs_trial{i_trial}}, {obj.ColorCommanded, obj.ColorTrials}, display)
                else
                    sgtitle(display, 'Aceleraciones articulares de la trayectoria comandada y ejecutada min, med y max');
                    IiwaPlotter.fill_joint_acceleration(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
                    IiwaPlotter.joint_accelerations({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials, obj.ColorTrials, obj.ColorCommanded, '--'}, display)
                    display.Children(1).String={'output', 'min', 'mean', 'max', 'commanded'};
                end
            end
        end
        function PlotCartesianPositionTrajectories(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            if (obj.ROSConnected)
                if (i_trial~=0)
                    sgtitle(display, ['Posiciones cartesianas de la trayectoria comandada y ejecutada', num2str(i_trial)]);
                    IiwaPlotter.cartesian_positions({obj.traj_command, obj.trajs_trial{i_trial}}, {obj.ColorCommanded, obj.ColorTrials}, display)
                else
                    sgtitle(display, 'Posiciones cartesianas de la trayectoria comandada y ejecutada mín, med y máx');
                    IiwaPlotter.fill_cartesian_position(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
                    IiwaPlotter.cartesian_positions({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials, obj.ColorTrials, obj.ColorCommanded, '--'}, display)
                    display.Children(1).String={'output', 'min', 'mean', 'max', 'commanded'};
                end
            end
        end
        function PlotCartesianVelocityTrajectories(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            if (obj.ROSConnected)
                if (i_trial~=0)
                    sgtitle(display, ['Velocidades cartesianas de la trayectoria comandada y ejecutada', num2str(i_trial)]);
                    IiwaPlotter.cartesian_velocities({obj.traj_command, obj.trajs_trial{i_trial}}, {obj.ColorCommanded, obj.ColorTrials}, display)
                else
                    sgtitle(display, 'Velocidades cartesianas de la trayectoria comandada y ejecutada mín, med y máx');
                    IiwaPlotter.fill_cartesian_velocity(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
                    IiwaPlotter.cartesian_velocities({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials, obj.ColorTrials, obj.ColorCommanded, '--'}, display)
                    display.Children(1).String={'output', 'min', 'mean', 'max', 'commanded'};
                end
            end
        end
        function Plot3DCartesianPositionTrajectories(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            if (obj.ROSConnected)
                if (i_trial~=0)
                    sgtitle(display, ['Posiciones cartesianas 3D de la trayectoria comandada y ejecutada', num2str(i_trial)]);
                    IiwaPlotter.cartesian_positions3d({obj.traj_command, obj.trajs_trial{i_trial}}, {obj.ColorCommanded, obj.ColorTrials}, display)
                else
                    for i=1:size(obj.trajs_trial,2)
                        sgtitle(display, ['Posiciones cartesianas 3D de todas las trayectoria ejecutada', num2str(i_trial)]);
                        IiwaPlotter.cartesian_positions3d(obj.trajs_trial(i), obj.ColorTrials, display);
                    end
                end
            end
        end
        % Plot error (dataCommanded - DataTrial) or (dataInput - dataCommanded)
        function PlotJointPositionError(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            if (obj.ROSConnected)
                if (i_trial~=0)
                    sgtitle(display, ['Error de posicion articular entre la trayectoria comandada y la ejecutada', num2str(i_trial)]);
                    IiwaPlotter.joint_position_error(obj.traj_command, obj.trajs_trial{i_trial}, obj.ColorTrials, display);
                else
                    sgtitle(display, 'Error de posicion articular min, med y max entre la comandada y cada ejecutada');
                    IiwaPlotter.fill_joint_position(obj.traj_min_error, obj.traj_max_error, obj.ColorTrials, display);
                    IiwaPlotter.joint_positions({obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
                    IiwaPlotter.joint_position_rms(obj.traj_mean_error, obj.ColorTrials, display);
                    display.Children(1).String={'error', 'min', 'mean', 'max', 'RMS'};
                end
            end
        end
        function PlotJointVelocityError(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            if (obj.ROSConnected)
                if (i_trial~=0)
                    sgtitle(display, ['Error de velocidad articular entre la trayectoria comandada y la ejecutada', num2str(i_trial)]);
                    IiwaPlotter.joint_velocity_error(obj.traj_command, obj.trajs_trial{i_trial}, obj.ColorTrials, display);
                else
                    sgtitle(display, 'Error de velocidad articular min, med y max entre la comandada y cada ejecutada');
                    IiwaPlotter.fill_joint_velocity(obj.traj_min_error, obj.traj_max_error, obj.ColorTrials, display);
                    IiwaPlotter.joint_velocities({obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
                    IiwaPlotter.joint_velocity_rms(obj.traj_mean_error, obj.ColorTrials, display);
                    display.Children(1).String={'error', 'min', 'mean', 'max', 'RMS'};
                end
            end
        end
        function PlotJointAccelerationError(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            if (obj.ROSConnected)
                if (i_trial~=0)
                    sgtitle(display, ['Error de aceleración articular entre la trayectoria comandada y la ejecutada', num2str(i_trial)]);
                    IiwaPlotter.joint_acceleration_error(obj.traj_command, obj.trajs_trial{i_trial}, obj.ColorTrials, display);
                else
                    sgtitle(display, 'Error de aceleración articular min, med y max entre la comandada y cada ejecutada');
                    IiwaPlotter.fill_joint_acceleration(obj.traj_min_error, obj.traj_max_error, obj.ColorTrials, display);
                    IiwaPlotter.joint_accelerations({obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
                    IiwaPlotter.joint_acceleration_rms(obj.traj_mean_error, obj.ColorTrials, display);
                    display.Children(1).String={'error', 'min', 'mean', 'max', 'RMS'};
                end    
            end
        end
        function PlotCartesianPositionError(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            if (obj.ROSConnected)
                if (i_trial~=0)
                    sgtitle(display, ['Error de posicion cartesiana entre la trayectoria comandada y la ejecutada', num2str(i_trial)]);
                    IiwaPlotter.cartesian_position_error(obj.traj_command, obj.trajs_trial{i_trial}, display);
                else
                    sgtitle(display, 'Error de posicion cartesiana min, med y max entre la trayectoria comandada y cada ejecutada');
                    IiwaPlotter.cartesian_repeatibility_position_error(obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error,  display)
                end
            end
        end
        function PlotCartesianVelocityError(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            if (obj.ROSConnected)
                if (i_trial~=0)
                    sgtitle(display, ['Error de velocidad cartesiana entre la trayectoria comandada y la ejecutada', num2str(i_trial)]);
                    IiwaPlotter.cartesian_velocity_error(obj.traj_command, obj.trajs_trial{i_trial}, display);
                else
                    sgtitle(display, 'Error de velocidad cartesiana min, med y max entre la trayectoria comandada y cada ejecutada');
                    IiwaPlotter.cartesian_repeatibility_velocity_error(obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error,  display)
                end
            end
        end
        % Plot repeatibility errors (DataTrialMean - DataTrial) -> Use only to compare multiple trials 
        function PlotJointPositionRepeatibilityError(obj, display, varargin)
            if (obj.Outside==1)
                display = figure;
            end
            if(obj.ROSConnected)
                sgtitle(display, 'Error de posicion articular min, med y max entre la ejecutada media y cada ejecutada');
                IiwaPlotter.fill_joint_position(obj.traj_min_error_rep, obj.traj_max_error_rep, obj.ColorTrials, display);
                IiwaPlotter.joint_positions({obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
                IiwaPlotter.joint_position_rms(obj.traj_mean_error_rep, obj.ColorTrials, display);
                display.Children(1).String={'error', 'min', 'mean', 'max', 'RMS'};
            end
        end
        function PlotJointVelocityRepeatibilityError(obj, display, varargin)
            if (obj.Outside==1)
                display = figure;
            end
           if(obj.ROSConnected)
                sgtitle(display, 'Error de velocidad articular min, med y max entre la ejecutada media y cada ejecutada');
                IiwaPlotter.fill_joint_velocity(obj.traj_min_error_rep, obj.traj_max_error_rep, obj.ColorTrials, display);
                IiwaPlotter.joint_velocities({obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
                IiwaPlotter.joint_velocity_rms(obj.traj_mean_error_rep, obj.ColorTrials, display);
                display.Children(1).String={'error', 'min', 'mean', 'max', 'RMS'};
            end
        end
        function PlotJointAccelerationRepeatibilityError(obj, display, varargin)
            if (obj.Outside==1)
                display = figure;
            end
            if(obj.ROSConnected)
                sgtitle(display, 'Error de aceleración articular min, med y max entre la ejecutada media y cada ejecutada');
                IiwaPlotter.fill_joint_acceleration(obj.traj_min_error_rep, obj.traj_max_error_rep, obj.ColorTrials, display);
                IiwaPlotter.joint_accelerations({obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
                IiwaPlotter.joint_acceleration_rms(obj.traj_mean_error_rep, obj.ColorTrials, display);
                display.Children(1).String={'error', 'min', 'mean', 'max', 'RMS'};
            end
        end
        function PlotCartesianPositionRepeatibilityError(obj, display, varargin)
            if (obj.Outside==1)
                display = figure;
            end
            if(obj.ROSConnected)
                sgtitle(display, 'Error de posicion cartesiana min, med y max entre la trayectoria ejecutada media y cada ejecutada');
                IiwaPlotter.cartesian_repeatibility_position_error(obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep,  display)
            end
        end
        function PlotCartesianVelocityRepeatibilityError(obj, display, varargin)
            if (obj.Outside==1)
                display = figure;
            end
            if(obj.ROSConnected)
                sgtitle(display, 'Error de velocidad cartesiana min, med y max entre la trayectoria ejecutada media y cada ejecutada');
                IiwaPlotter.cartesian_repeatibility_velocity_error(obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep,  display)
            end
        end
        function PlotJointTorques(obj, i_trial, display)
            if (obj.Outside==1)
                display=figure;
            end
            if (obj.ROSConnected)
                if (i_trial~=0)
                    sgtitle(display, ['Pares articulares sentidos en la trayectoria de referencia y la ejecutada', num2str(i_trial)]);
                    IiwaPlotter.joint_efforts(obj.traj_reference, obj.ColorCommanded, display);
                    IiwaPlotter.joint_efforts(obj.trajs_trial{i_trial}, obj.ColorTrials, display);
                    display.Children(1).String={'reference', 'output'};
                end
            end
        end
        function PlotCartesianForceTorque(obj, i_trial, display, varargin)
            if (obj.Outside==1)
            end
            if (obj.ROSConnected)
                sgtitle(display, ['Fuerza-Par cartesiano en la herramienta durante la trayectoria ejecutada', num2str(i_trial)]);
                npoints_used=min(obj.traj_reference.npoints, obj.trajs_trial{i_trial}.npoints);
                tr_trial = obj.trajs_trial{i_trial}.GetFirstNPoints(npoints_used);
                eff_ref = obj.traj_reference.GetFirstNPoints(npoints_used).effort;
                eff_trial = tr_trial.effort;
                eff_diff = eff_trial - eff_ref;
                traj_ft = IiwaTrajectory(npoints_used);
                for i=1:npoints_used
                    Jst = GeoJacobianS([IiwaParameters.Twist; tr_trial.q(i,:)]);
                    FT(i,:) = Jst*eff_diff(i,:)';
                end
                FT = FT./100; %La gráfica mostrará "cm", multiplicando los valores leídos por 100, por lo que se debe dividir por 100 antes.
                traj_ft.x=FT;
                traj_ft.t=tr_trial.t;
                traj_ft.name='FT';
                IiwaPlotter.cartesian_positions(traj_ft, obj.ColorTrials, display);
                for i=1:size(display.Children,1)
                    if (isa(display.Children(i), 'matlab.graphics.axis.Axes'))
                        if (~strcmp(display.Children(i).YLabel.String, '[deg]'))
                            display.Children(i).YLabel.String = '$N$';
                        else
                            display.Children(i).YLabel.String = '$N\cdot m$';
                        end
                    end
                end
            end
        end
    end
end

