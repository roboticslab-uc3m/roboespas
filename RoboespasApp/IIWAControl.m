classdef IIWAControl < handle
    properties(Constant)
        Outside = 0;
         %Capture trajectory initial positions
        ReferencePositionRight= [-1.7910    1.1126   -2.1927    1.8522   -1.2179   -0.8831   -0.2125];%-1.428907871246338   1.280634284019470  -2.738452911376953   1.561375141143799  -0.955803275108337  -0.486742466688156  -0.421205967664719];
        ReferencePositionLeft = [  -2.124842166900635   1.256874322891235  -2.738421201705933   1.675831675529480  -1.081720232963562  -0.424112081527710  -0.960969746112823];

        %Plot specifications
        ColorCommanded=[0.93, 0.69, 0.13];
        ColorTrials=[0.85, 0.33, 0.10];
        ColorInput='g';
%         ColorCommanded='b';
%         ColorTrials='r';
        
        %File save specifications
        AdjustedIdentifier='adj';
        InputIdentifier='cap';
        TrajectoriesFolder = '/home/roboespas/roboespas/Common/Trajectories'
        
        CommandMode = 'Stack' %''FRI' %
    end
    properties(Access=private)
         %Properties for enabling not-connected mode
        ROSConnected = 0
        DefaultCapTrajectoryList = 'flexext_P001_L_000000_0000_cap'% Description
        DefaultAdjTrajectoryList = 'flexext_P001_L_000000_0000_adj'% Description
    end
    
    properties(Access=public)
        %File save specifications
        NameMovement = 'flexext'
        
        %Object used to control robot
        IiwaCommand %Can be IiwaCommandStack, IiwaCommandFri or IiwaCommandGazebo
        
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
        CommandedSampleTime = 0.005 %Seconds
    end

    methods
%% CONSTRUCTOR
        function obj = IIWAControl(ConnectROS)
            %addpath(genpath(fileparts(which(mfilename))));
            %savepath;
            if (ConnectROS==1)
                init_ros;
                obj.ROSConnected=1;
                clear IiwaCommandStack;
                clear IiwaMsgTransformer;
                if (strcmp(obj.CommandMode,'Stack'))
                    obj.IiwaCommand = IiwaCommandStack;
                elseif(strcmp(obj.CommandMode, 'FRI'))
                    obj.IiwaCommand = IiwaCommandFRI;
                else
                    ME=MException('IIWAControl:IiwaCommandMode', 'IIWAControl.CommandMode must be "FRI" or "Stack"');
                    throw(ME);
                end
                %Set maximum velocity, acceleration and jerk
                obj.IiwaCommand.SetVelocity(1);
                obj.IiwaCommand.SetAcceleration(1);
                obj.IiwaCommand.SetJerk(1);
                obj.CommandedSampleTime = obj.IiwaCommand.GetControlStepSize();
            else
                obj.ROSConnected=0;
            end
        end
%% PUBLIC METHODS
        % ROBOT MOVEMENT FUNCTIONS
        function SendReferencePositionLeft(obj)
            if (obj.ROSConnected)
                IiwaCommandFRI.SetVelocity(0.1);
            	[traj_comm, traj_out] = obj.IiwaCommand.MoveJ(obj.ReferencePositionLeft);
                %IiwaPlotter.joint_position_error(traj_out, {traj_theory, traj_comm}, ['r', 'b']);
            end
        end
        function SendReferencePositionRight(obj)
            if (obj.ROSConnected)
                [traj_comm, traj_out] = obj.IiwaCommand.MoveJ(obj.ReferencePositionRight);
                %IiwaPlotter.joint_position_error(traj_comm, traj_out, 'r');
            end
        end
        function SendInitialPosition(obj)
           	if (obj.ROSConnected)
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
                [traj_command, obj.traj_reference] = obj.IiwaCommand.SendTraj(obj.traj_mirrored);
            else
                pause(5);
            end
        end
        function [IDPatient, Arm] = SendTrialTrajectory(obj, i_trial)
            [IDPatient, Arm] = obj.SetNameData(obj.NameData);
            if (obj.ROSConnected)
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
            if (obj.ROSConnected)
                listing = dir(obj.TrajectoriesFolder);
                InputList = {};
                n=1;
                for i=1:length(listing)
                    if (strcmp(listing(i).name, '.')~=1 && strcmp(listing(i).name, '..')~=1)
                        if(~isempty(strfind(listing(i).name, obj.InputIdentifier)))
                            InputList{n} = listing(i).name; %#ok<AGROW>
                            n=n+1;
                        end
                    end
                end
            else
                InputList = obj.DefaultCapTrajectoryList;
            end
        end
        function [IDPatient, Arm] = LoadInput(obj, name_input) 
            [IDPatient, Arm] = obj.SetNameData(name_input(1:strfind(name_input, obj.InputIdentifier)-1));
            %Load data input
            if (obj.ROSConnected)
                obj.traj_input = obj.LoadTrajectory(obj.NameDataInput, 'input');
                obj.traj_input_no_pauses = obj.traj_input.DeleteInitialEndPauses();
            end
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
        	folder = [obj.TrajectoriesFolder, '/', folder, '/'];
            mkdir(folder);
            angvel = obj.circle_angvel;
            save([folder, traj.name, '.mat'], 'traj', 'angvel');
        end
        function traj = LoadTrajectory(obj, name_folder, name)
            folder = [obj.TrajectoriesFolder, '/', name_folder, '/'];
            traj = load([folder, name, '.mat']);
            if (~isempty(traj.angvel))
                obj.circle_angvel = traj.angvel;
            end
            traj = traj.traj;
        end
        % DATA ADJUSTED/COMMANDED FUNCTIONS
        function AdjustedList = DataAdjustedList(obj)
            if (obj.ROSConnected)
                listing = dir(obj.TrajectoriesFolder);
                AdjustedList = {};
                n=1;
                for i=1:length(listing)
                    if (strcmp(listing(i).name, '.')~=1 && strcmp(listing(i).name, '..')~=1)
                        if(~isempty(strfind(listing(i).name, obj.AdjustedIdentifier)))
                            AdjustedList{n} = listing(i).name; %#ok<AGROW>
                            n=n+1;
                        end
                    end
                end
            else
                AdjustedList = obj.DefaultAdjTrajectoryList;
            end
        end
        function [IDPatient, Arm] = LoadAdjusted(obj, name_commanded) 
            [IDPatient, Arm] = obj.SetNameData(name_commanded(1:strfind(name_commanded, obj.AdjustedIdentifier)-1));
            %Load data input
            if (obj.ROSConnected)
                obj.traj_adjusted = obj.LoadTrajectory(obj.NameDataAdjusted, 'adjusted');
            end
        end
        function [IDPatient, Arm] = BuildDataAdjusted(obj, name_input)
            [IDPatient, Arm] = obj.SetNameData(name_input(1:strfind(name_input, obj.InputIdentifier)-1));
            if (obj.ROSConnected)
                obj.traj_resampled = obj.traj_input.ChangeSampleTime(obj.CommandedSampleTime);
                obj.traj_input_no_pauses = obj.traj_resampled.DeleteInitialEndPauses();
                [obj.traj_input_circle, C, R, arc, obj.circle_angvel] = obj.traj_input_no_pauses.FitToCircle('X');

                obj.traj_adjusted = obj.traj_input_circle;
                obj.traj_adjusted.name = 'adjusted';
                obj.SaveTrajectory(obj.traj_adjusted, obj.NameDataAdjusted);
            else
                obj.DefaultAdjTrajectoryList=[obj.DefaultAdjTrajectoryList, {obj.NameDataAdjusted}];
            end
        end
        function ChangeVelocity(obj, w_out)
            obj.VelocityFactor=w_out;
            if (obj.ROSConnected)
                if (~isempty(obj.traj_adjusted))
                    obj.CommandedSampleTime = IiwaCommandFRI.GetControlStepSize;
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
        end
        % DATA OUTPUT FUNCTIONS
        function Trajectory = GetStruct(obj, i_trial)
            %%ADded as it is
            if (obj.ROSConnected)
                ppsec=3;
                v=obj.VelocityFactor;
                Trajectory.CapturedDate = obj.CapturedDate;
                DataReference = traj2Data(obj.traj_reference);
                Trajectory.Reference.Timestamps = DataReference.t;
                Trajectory.Reference.JointTrajectory = DataReference.q;
                Trajectory.Reference.CartesianTrajectory.Position = DataReference.x.pos;
                Trajectory.Reference.CartesianTrajectory.Orientation = DataReference.x.ori;
                Trajectory.Reference.JointVelocities = DataReference.qdot;
                Trajectory.Reference.CartesianVelocities.Position = DataReference.xdot.pos;
                Trajectory.Reference.CartesianVelocities.Orientation = DataReference.xdot.ori;
                Trajectory.Reference.JointAccelerations = DataReference.qdotdot;
                Trajectory.Reference.JointTorques.Timestamps = DataReference.t_torque;
                Trajectory.Reference.JointTorques.Torques = DataReference.q_torque;
                DataTrial = traj2Data(obj.trajs_trial{i_trial});
                Trajectory.Trial.Timestamps = DataTrial.t;
                Trajectory.Trial.JointTrajectory = DataTrial.q;
                Trajectory.Trial.CartesianTrajectory.Position = DataTrial.x.pos;
                Trajectory.Trial.CartesianTrajectory.Orientation = DataTrial.x.ori;
                Trajectory.Trial.JointVelocities = DataTrial.qdot;
                Trajectory.Trial.CartesianVelocities.Position = DataTrial.xdot.pos;
                Trajectory.Trial.CartesianVelocities.Orientation = DataTrial.xdot.ori;
                Trajectory.Trial.JointAccelerations = DataTrial.qdotdot;
                Trajectory.Trial.JointTorques.Timestamps = DataTrial.t_torque;
                Trajectory.Trial.JointTorques.Torques = DataTrial.q_torque;
                DataCommand = traj2Data(obj.traj_command);
                Trajectory.DataCommanded = DataCommand;
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
        function SetCommandedSampleTime(obj, sample_time)
            obj.CommandedSampleTime = sample_time;
            obj.IiwaCommand.SetControlStepSize(obj.CommandedSampleTime);
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
                    IiwaPlotter.joint_positions({obj.traj_command, obj.trajs_trial{i_trial}}, {obj.ColorInput, obj.ColorCommanded, obj.ColorTrials}, display)
                else
%                     IiwaPlotter.fill_joint_position(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
%                     IiwaPlotter.joint_positions({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials, obj.ColorTrials, obj.ColorCommanded, '--'}, display)
                    %q_command
%                     display = figure(1);
%                     IiwaPlotter.fill_joint_position(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
%                     IiwaPlotter.joint_positions({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials, obj.ColorTrials, obj.ColorCommanded, '--'}, display)
%                     %q_precision
%                     display = figure(2);
%                     IiwaPlotter.fill_joint_position(obj.traj_min_error, obj.traj_max_error, obj.ColorTrials, display);
%                     IiwaPlotter.joint_positions({obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
%                     IiwaPlotter.joint_position_rms(obj.traj_mean_error, obj.ColorTrials, display);
%                     %q_repeatibility
%                     display = figure(3);
%                     IiwaPlotter.fill_joint_position(obj.traj_min_error_rep, obj.traj_max_error_rep, obj.ColorTrials, display);
%                     IiwaPlotter.joint_positions({obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
%                     IiwaPlotter.joint_position_rms(obj.traj_mean_error_rep, obj.ColorTrials, display);
%                     %qd_command
%                     display = figure(4);
%                     IiwaPlotter.fill_joint_velocity(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
%                     IiwaPlotter.joint_velocities({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials,obj.ColorTrials, obj.ColorCommanded, '--'}, display)
%                     %qd_precision
%                     display = figure(5);
%                     IiwaPlotter.fill_joint_velocity(obj.traj_min_error, obj.traj_max_error, obj.ColorTrials, display);
%                     IiwaPlotter.joint_velocities({obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
%                     IiwaPlotter.joint_velocity_rms(obj.traj_mean_error, obj.ColorTrials, display);
%                     %qd_repeatibility
%                     display = figure(6);
%                     IiwaPlotter.fill_joint_velocity(obj.traj_min_error_rep, obj.traj_max_error_rep, obj.ColorTrials, display);
%                     IiwaPlotter.joint_velocities({obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
%                     IiwaPlotter.joint_velocity_rms(obj.traj_mean_error_rep, obj.ColorTrials, display);
%                     %x_command
%                     display = figure(7);
%                     IiwaPlotter.fill_cartesian_position(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
%                     IiwaPlotter.cartesian_positions({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials, obj.ColorTrials, obj.ColorCommanded, '--'}, display)
%                     %x_precision
%                     display = figure(8);
%                     IiwaPlotter.cartesian_repeatibility_position_error(obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error,  display)
%                     %x_repeatibility
%                     display = figure(9);
%                     IiwaPlotter.cartesian_repeatibility_position_error(obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep,  display)
%                     %xd_command
%                     display = figure(10);
%                     IiwaPlotter.fill_cartesian_velocity(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
%                     IiwaPlotter.cartesian_velocities({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials, obj.ColorTrials, obj.ColorCommanded, '--'}, display)
%                     %xd_precision
%                     display = figure(11);
%                     IiwaPlotter.cartesian_repeatibility_velocity_error(obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error,  display)
%                     %xd_repeatibility
%                     display = figure(12);
%                     IiwaPlotter.cartesian_repeatibility_velocity_error(obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep,  display)
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
                    IiwaPlotter.joint_velocities({obj.traj_command, obj.trajs_trial{i_trial}, obj.traj_mirrored}, {obj.ColorInput, obj.ColorCommanded, obj.ColorTrials, '.'}, display)
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
                    IiwaPlotter.joint_accelerations({obj.traj_command, obj.trajs_trial{i_trial}}, {obj.ColorInput, obj.ColorCommanded, obj.ColorTrials}, display)
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
                    IiwaPlotter.cartesian_positions({obj.traj_command, obj.trajs_trial{i_trial}}, {obj.ColorInput, obj.ColorCommanded, obj.ColorTrials}, display)
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
                    IiwaPlotter.cartesian_velocities({obj.traj_command, obj.trajs_trial{i_trial}}, {obj.ColorInput, obj.ColorCommanded, obj.ColorTrials}, display)
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
                    IiwaPlotter.cartesian_positions3d({obj.traj_command, obj.trajs_trial{i_trial}}, {obj.ColorInput, obj.ColorCommanded, obj.ColorTrials}, display)
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
                    %TODO: Add Euclidean
                    IiwaPlotter.cartesian_repeatibility_position_error(obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error,  display)
                    %IiwaPlotter.fill_cartesian_position(obj.traj_min_error, obj.traj_max_error, obj.ColorTrials, display);
                    %IiwaPlotter.cartesian_positions({obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
                    %IiwaPlotter.cartesian_position_rms(obj.traj_mean_error, obj.ColorTrials, display);
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
                    %TODO: Add Euclidean
                    IiwaPlotter.cartesian_repeatibility_velocity_error(obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error,  display)
%                     IiwaPlotter.fill_cartesian_velocity(obj.traj_min_error, obj.traj_max_error, obj.ColorTrials, display);
%                     IiwaPlotter.cartesian_velocities({obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error}, {obj.ColorTrials, 'k', obj.ColorTrials}, display)  
%                     IiwaPlotter.cartesian_velocity_rms(obj.traj_mean_error, obj.ColorTrials, display);
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
                %TODO: Add Euclidean
                IiwaPlotter.cartesian_repeatibility_position_error(obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep,  display)
                %IiwaPlotter.fill_cartesian_position(obj.traj_min_error_rep, obj.traj_max_error_rep, obj.ColorTrials, display);
                %IiwaPlotter.cartesian_positions({obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
                %IiwaPlotter.cartesian_position_rms(obj.traj_mean_error_rep, obj.ColorTrials, display);
            end
        end
        function PlotCartesianVelocityRepeatibilityError(obj, display, varargin)
            if (obj.Outside==1)
                display = figure;
            end
            if(obj.ROSConnected)
                %sgtitle(display, 'Cartesian Velocity Error Repeatibility Mean - Trials');
                %TODO: Add Euclidean
                IiwaPlotter.cartesian_repeatibility_velocity_error(obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep,  display)
%                 IiwaPlotter.fill_cartesian_velocity(obj.traj_min_error_rep, obj.traj_max_error_rep, obj.ColorTrials, display);
%                 IiwaPlotter.cartesian_velocities({obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
%                 IiwaPlotter.cartesian_velocity_rms(obj.traj_mean_error_rep, obj.ColorTrials, display);
            end
        end 
    end
end

