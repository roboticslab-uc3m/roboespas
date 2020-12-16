classdef IIWAControl < handle
    properties(Constant)
        Outside = 0;
         %Capture trajectory initial positions
        ReferencePositionRight = [-1.7910, 1.1126, -2.1927, 1.8522, -1.2179, -0.8831, -0.2125];
        ReferencePositionLeft = [-2.124842166900635, 1.256874322891235, -2.738421201705933, 1.675831675529480, -1.081720232963562, -0.424112081527710, -0.960969746112823];

        %Plot specifications
        ColorCommanded=[0.93, 0.69, 0.13];
        ColorTrials=[0.85, 0.33, 0.10];
        ColorInput='g';
        
        %File save specifications
        AdjustedIdentifier='adj';
        InputIdentifier='cap';
        TrajectoriesFolder = '/home/roboespas/roboespas/RoboespasApp/Results/IIWATrajectories/CurrentTrajectories';
        
        CommandedSampleTime = 0.005 %Seconds
    end
    properties(Access=private)
        %Properties for enabling not-connected mode
        ROSConnected = 0
    end
    
    properties(Access=public)
        %File save specifications
        NameMovement = 'flexext'
        
        %Object used to control robot
        CommandMode = 'Stack' % 'Stack' or 'FRI', stablished automatically depending on the existing ROS nodes. If ROS not connected, 'Stack' is chosen, although it does not matter
        IiwaCommand % Object of type IiwaCommandStack or IiwaCommandFRI, depending on CommandMode
        
        %TRAJECTORIES
        %Captured input trajectories
        traj_input
        traj_resampled
        traj_input_no_pauses
        traj_input_circle
        traj_input_show
        circle_angvel
        %Adjusted trajectories
        traj_velocity_circle
        traj_spline
        traj_mirrored
        traj_adjusted
        %Commanded trajectories - after adjusting velocity
        traj_command
        VelocityFactor
        CommandedAngularVelocity
        %Output trajectories
        trajs_trial={};
        traj_reference
 
        %Repeatibility properties
        traj_min
        traj_mean
        traj_max
        
        traj_min_error
        traj_mean_error
        traj_max_error
        
        traj_min_error_rep
        traj_mean_error_rep
        traj_max_error_rep     
    end
    
    properties(GetAccess=public, SetAccess=private)
        NameData
        NameDataInput
        NameDataAdjusted
        CapturedDate
        
        %Trajectory adjustment parameters
        CommandedMiddlePause = 2; %Seconds
        nSegCommand = 50;
        SmoothingCommand = 1e-5;
        
        %Circle error admitted
        MaxCircleError = 0.04;
    end

    methods
%% CONSTRUCTOR
        function obj = IIWAControl()
            %addpath(genpath(fileparts(which(mfilename))));
            %savepath;
            obj.ROSConnected=0;
            IiwaPlotter.Initialize();
        end
%% PUBLIC METHODS
        function CommandMode = ConnectROS(obj)
            init_ros;
            obj.ROSConnected=1;
            clear IiwaCommandStack;
            clear IiwaCommandFRI;
            clear IiwaMsgTransformer;
           
            if (contains(cell2mat(rosnode('list')), 'stack'))
                obj.CommandMode = 'Stack';
                obj.IiwaCommand = IiwaCommandStack;
            elseif(contains(cell2mat(rosnode('list')), 'FRI'))
                obj.CommandMode = 'FRI';
                obj.IiwaCommand = IiwaCommandFRI;
            else
                ME=MException('IIWAControl:IiwaCommandMode', 'IIWAControl.CommandMode must be "FRI" or "Stack"');
                throw(ME);
            end
            %Set maximum velocity, acceleration and jerk
            obj.IiwaCommand.SetVelocity(1);
            obj.IiwaCommand.SetAcceleration(1);
            obj.IiwaCommand.SetJerk(1);
            CommandMode = obj.CommandMode;
        end
        % ROBOT MOVEMENT FUNCTIONS
        function SendReferencePositionLeft(obj)
            if (obj.ROSConnected)
                obj.IiwaCommand.SetVelocity(0.1);
            	[traj_comm, traj_out] = obj.IiwaCommand.MoveJ(obj.ReferencePositionLeft);
            end
        end
        function SendReferencePositionRight(obj)
            if (obj.ROSConnected)
                obj.IiwaCommand.SetVelocity(0.1);
                [traj_comm, traj_out] = obj.IiwaCommand.MoveJ(obj.ReferencePositionRight);
            end
        end
        function SendInitialPosition(obj)
           	if (obj.ROSConnected)
                obj.IiwaCommand.SetVelocity(1);
                if (~isempty(obj.traj_mirrored))
                    obj.IiwaCommand.MoveJ(obj.traj_mirrored.q(1,:));
                else
                    ME=MException('IIWAControl:EmptyDataCommand', 'traj_command vacio');
                    throw(ME);
                end
            end
        end
        function [IDPatient, Arm] = SendReferenceTrajectory(obj)
            [IDPatient, Arm] = obj.SetNameData(obj.NameData);
            if (obj.ROSConnected)
                obj.IiwaCommand.SetVelocity(1);
                [traj_command, obj.traj_reference] = obj.IiwaCommand.SendTraj(obj.traj_mirrored);
            else
                pause(5);
            end
        end
        function [IDPatient, Arm] = SendTrialTrajectory(obj, i_trial)
            [IDPatient, Arm] = obj.SetNameData(obj.NameData);
            if (obj.ROSConnected)
                obj.IiwaCommand.SetVelocity(1);
                [obj.traj_command, obj.trajs_trial{i_trial}] = obj.IiwaCommand.SendTraj(obj.traj_mirrored);
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
            [obj.traj_input_circle, C, R, arc, obj.circle_angvel, e_rms] = obj.traj_input_no_pauses.FitToCircle('X', display);
            obj.traj_adjusted = obj.traj_input_circle;
            obj.traj_adjusted.name = 'adjusted';
            obj.SaveTrajectory(obj.traj_adjusted, obj.NameDataAdjusted);
        end
        
        function ChangeVelocity(obj, w_out)
            obj.VelocityFactor=w_out;
            if (~isempty(obj.traj_adjusted))
                obj.traj_velocity_circle = obj.traj_adjusted.ChangeVelocity(w_out/obj.circle_angvel);
                obj.traj_spline = obj.traj_velocity_circle.BoundedSpline(obj.SmoothingCommand, obj.nSegCommand);
                traj_spline_pause = obj.traj_spline.AddPause(obj.CommandedMiddlePause);
                obj.traj_mirrored = traj_spline_pause.MergeAfterwards(obj.traj_spline.MirrorTrajectory());
                obj.traj_mirrored.name = 'mirrored';
                obj.traj_mirrored = obj.traj_mirrored.FixCartesianCoordinates('XBC'); %For commanding cartesian velocities
                if (~isempty(obj.traj_input))
                    obj.traj_input_show = obj.traj_input_no_pauses.ChangeVelocity(w_out/obj.circle_angvel);
                else
                    ME=MException('IIWAControl:EmptyTrajInput', 'traj_input esta vacio');
                    throw(ME);
                end
            else
                ME=MException('IIWAControl:EmptyTrajAdjusted', 'traj_adjusted esta vacio');
                throw(ME);
            end
        end
        % DATA OUTPUT FUNCTIONS
        function Trajectory = GetStruct(obj, i_trial)
            %%ADded as it is
            if (obj.ROSConnected)
                ppsec=3;
                v=obj.VelocityFactor;
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
            if (contains(name_data, '_L_'))
                Arm='L';
            elseif (contains(name_data, '_R_'))
                Arm='R';
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
            %sgtitle(display, '3D Cartesian Position');
            IiwaPlotter.cartesian_positions3d({obj.traj_velocity_circle, obj.traj_spline, obj.traj_mirrored}, ['b', 'r', 'g', 'm'], display);     
        end
        function PlotJointPositionTrajectories(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            %sgtitle(display, 'Joint Position');
            if (obj.ROSConnected)
                if (i_trial~=0)
                    IiwaPlotter.joint_positions({obj.traj_command, obj.trajs_trial{i_trial}}, {obj.ColorCommanded, obj.ColorTrials}, display)
                else
                    IiwaPlotter.fill_joint_position(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
                    IiwaPlotter.joint_positions({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials, obj.ColorTrials, obj.ColorCommanded, '--'}, display)
                end
            end
        end
        function PlotJointVelocityTrajectories(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            %sgtitle(display, 'Joint velocity');
            if (obj.ROSConnected)
                if (i_trial~=0)
                    IiwaPlotter.joint_velocities({obj.traj_command, obj.trajs_trial{i_trial}}, {obj.ColorCommanded, obj.ColorTrials, '.'}, display)
                else
                    IiwaPlotter.fill_joint_velocity(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
                    IiwaPlotter.joint_velocities({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials,obj.ColorTrials, obj.ColorCommanded, '--'}, display)
                end
            end
        end
        function PlotJointAccelerationTrajectories(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            %sgtitle(display, 'Joint acceleration');
            if (obj.ROSConnected)
                if (i_trial~=0)
                    IiwaPlotter.joint_accelerations({obj.traj_command, obj.trajs_trial{i_trial}}, {obj.ColorCommanded, obj.ColorTrials}, display)
                else 
                    IiwaPlotter.fill_joint_acceleration(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
                    IiwaPlotter.joint_accelerations({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials, obj.ColorTrials, obj.ColorCommanded, '--'}, display)
                end
            end
        end
        function PlotCartesianPositionTrajectories(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            %sgtitle(display, 'Cartesian position');
            if (obj.ROSConnected)
                if (i_trial~=0)
                    IiwaPlotter.cartesian_positions({obj.traj_command, obj.trajs_trial{i_trial}}, {obj.ColorCommanded, obj.ColorTrials}, display)
                else
                    IiwaPlotter.fill_cartesian_position(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
                    IiwaPlotter.cartesian_positions({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials, obj.ColorTrials, obj.ColorCommanded, '--'}, display)
                end
            end
        end
        function PlotCartesianVelocityTrajectories(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            %sgtitle(display, 'Cartesian Velocity');
            if (obj.ROSConnected)
                if (i_trial~=0)
                    IiwaPlotter.cartesian_velocities({obj.traj_command, obj.trajs_trial{i_trial}}, {obj.ColorCommanded, obj.ColorTrials}, display)
                else
                    IiwaPlotter.fill_cartesian_velocity(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
                    IiwaPlotter.cartesian_velocities({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials, obj.ColorTrials, obj.ColorCommanded, '--'}, display)
                end
            end
        end
        function Plot3DCartesianPositionTrajectories(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            %sgtitle(display, '3D Cartesian Trajectory');
            if (obj.ROSConnected)
                if (i_trial~=0)
                    IiwaPlotter.cartesian_positions3d({obj.traj_command, obj.trajs_trial{i_trial}}, {obj.ColorCommanded, obj.ColorTrials}, display)
                else
                    IiwaPlotter.cartesian_positions3d(obj.trajs_trial, obj.ColorTrials*ones(1,size(obj.trajs_trial,2)), display);
                end
            end
        end
        % Plot error (dataCommanded - DataTrial) or (dataInput - dataCommanded)
        function PlotJointPositionError(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            %sgtitle(display, 'Joint Position Error');
            if (obj.ROSConnected)
                if (i_trial~=0)
                    IiwaPlotter.joint_position_error(obj.traj_command, obj.trajs_trial{i_trial}, obj.ColorTrials, display);
                else
                    %sgtitle(display, 'Joint Position Error Repeatibility Command - Trials');
                    IiwaPlotter.fill_joint_position(obj.traj_min_error, obj.traj_max_error, obj.ColorTrials, display);
                    IiwaPlotter.joint_positions({obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
                    IiwaPlotter.joint_position_rms(obj.traj_mean_error, obj.ColorTrials, display);
                end
            end
        end
        function PlotJointVelocityError(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            %sgtitle(display, 'Joint Velocity Error');
            if (obj.ROSConnected)
                if (i_trial~=0)
                    IiwaPlotter.joint_velocity_error(obj.traj_command, obj.trajs_trial{i_trial}, obj.ColorTrials, display);
                else
                    %sgtitle(display, 'Joint Velocity Error Repeatibility Command - Trials');
                    IiwaPlotter.fill_joint_velocity(obj.traj_min_error, obj.traj_max_error, obj.ColorTrials, display);
                    IiwaPlotter.joint_velocities({obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
                    IiwaPlotter.joint_velocity_rms(obj.traj_mean_error, obj.ColorTrials, display);
                end
            end
        end
        function PlotJointAccelerationError(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            %sgtitle(display, 'Joint Acceleration Error');
            if (obj.ROSConnected)
                if (i_trial~=0)
                    IiwaPlotter.joint_acceleration_error(obj.traj_command, obj.trajs_trial{i_trial}, obj.ColorTrials, display);
                else
                    %sgtitle(display, 'Joint Acceleration Error Repeatibility Command - Trials');
                    IiwaPlotter.fill_joint_acceleration(obj.traj_min_error, obj.traj_max_error, obj.ColorTrials, display);
                    IiwaPlotter.joint_accelerations({obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
                    IiwaPlotter.joint_acceleration_rms(obj.traj_mean_error, obj.ColorTrials, display);
                end    
            end
        end
        function PlotCartesianPositionError(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            %sgtitle(display, 'Cartesian Position Error');
            if (obj.ROSConnected)
                if (i_trial~=0)
                    IiwaPlotter.cartesian_position_error(obj.traj_command, obj.trajs_trial{i_trial}, display);
                else
                    %sgtitle(display, 'Cartesian Position Error Repeatibility Command - Trials');
                    IiwaPlotter.cartesian_repeatibility_position_error(obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error,  display)
                end
            end
        end
        function PlotCartesianVelocityError(obj, i_trial, display)
            if (obj.Outside==1)
                display = figure;
            end
            %sgtitle(display, 'Cartesian Velocity Error');
            if (obj.ROSConnected)
                if (i_trial~=0)
                    IiwaPlotter.cartesian_velocity_error(obj.traj_command, obj.trajs_trial{i_trial}, display);
                else
                    %sgtitle(display, 'Cartesian Velocity Error Repeatibility Command - Trials');
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
                %sgtitle(display, 'Joint Position Error Repeatibility Mean - Trials');
                IiwaPlotter.fill_joint_position(obj.traj_min_error_rep, obj.traj_max_error_rep, obj.ColorTrials, display);
                IiwaPlotter.joint_positions({obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
                IiwaPlotter.joint_position_rms(obj.traj_mean_error_rep, obj.ColorTrials, display);
            end
        end
        function PlotJointVelocityRepeatibilityError(obj, display, varargin)
            if (obj.Outside==1)
                display = figure;
            end
           if(obj.ROSConnected)
                %sgtitle(display, 'Joint Velocity Error Repeatibility Mean - Trials');
                IiwaPlotter.fill_joint_velocity(obj.traj_min_error_rep, obj.traj_max_error_rep, obj.ColorTrials, display);
                IiwaPlotter.joint_velocities({obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
                IiwaPlotter.joint_velocity_rms(obj.traj_mean_error_rep, obj.ColorTrials, display);
            end
        end
        function PlotJointAccelerationRepeatibilityError(obj, display, varargin)
            if (obj.Outside==1)
                display = figure;
            end
            if(obj.ROSConnected)
                %sgtitle(display, 'Joint Acceleration Error Repeatibility Mean - Trials');
                IiwaPlotter.fill_joint_acceleration(obj.traj_min_error_rep, obj.traj_max_error_rep, obj.ColorTrials, display);
                IiwaPlotter.joint_accelerations({obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
                IiwaPlotter.joint_acceleration_rms(obj.traj_mean_error_rep, obj.ColorTrials, display);
            end
        end
        function PlotCartesianPositionRepeatibilityError(obj, display, varargin)
            if (obj.Outside==1)
                display = figure;
            end
            if(obj.ROSConnected)
                %sgtitle(display, 'Cartesian Position Error Repeatibility Mean - Trials');
                IiwaPlotter.cartesian_repeatibility_position_error(obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep,  display)
            end
        end
        function PlotCartesianVelocityRepeatibilityError(obj, display, varargin)
            if (obj.Outside==1)
                display = figure;
            end
            if(obj.ROSConnected)
                %sgtitle(display, 'Cartesian Velocity Error Repeatibility Mean - Trials');
                IiwaPlotter.cartesian_repeatibility_velocity_error(obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep,  display)
            end
        end
        function PlotJointTorques(obj, i_trial, display)
            if (obj.Outside==1)
                display=figure;
            end
            if (obj.ROSConnected)
                if (i_trial~=0)
                    IiwaPlotter.joint_efforts(obj.trajs_trial{i_trial}, obj.ColorTrials, display);
                end
            end
        end
        function PlotCartesianForceTorque(obj, display, varargin)
            if (obj.Outside==1)
            end
            if (obj.ROSConnected)
                disp('a');
            end
        end
    end
end

