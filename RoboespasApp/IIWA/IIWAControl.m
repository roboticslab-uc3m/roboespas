classdef IIWAControl < handle
    % Summary of this class goes here
    %   Detailed explanation goes here
    properties(Constant)
        MasterIP  = '160.69.69.100';
        MasterPort = '11311';
        
        InputIdentifier='cap';
        
        CommandedSampleTime = 0.01 %Seconds
        CommandedMiddlePause = 2; %Seconds
        CommandedIdentifier='adj';
        
        %!!Trucadas ambas a right porque el robot est� pegado a la mesa
        ReferencePositionLeft = [ -1.43034565448761, 1.277533888816833, -2.738382339477539, 1.576292872428894, -1.003308653831482, -0.4687036871910095, -0.3838373422622681]; % %[-2.27228709005705;1.27528126290555;-2.76711361378742;1.61695719633329;-0.970375948889415;-0.409916398365001; -1.24954618283254]; %
        ReferencePositionRight = [ -1.43034565448761, 1.277533888816833, -2.738382339477539, 1.576292872428894, -1.003308653831482, -0.4687036871910095, -0.3838373422622681]; % 
        
        ThetaMax = deg2rad([170 120 170 120 170 120 175]);
        ThetaDotMax = deg2rad([67 67 81 81 130 120 120]);
        ThetaDotDotMax = deg2rad([0 0 0 0 0 0 0]);
        NameCoord = ['X', 'Y', 'Z', 'A', 'B', 'C'];  %Can be changed but must be different and one-character sized
        ColorsXYZ = ['b', 'r', 'g'];
        ColorsJoints = ['b', 'k', 'r', 'g', 'y', 'c', 'm'];
        ColorDataInput='g';
        ColorDataCommanded='b';
        ColorDataOutput='r';
        FigureLegend={'Grabada', 'Comandada', 'Ejecutada'}
        ColorLightBlue=[152/256, 163/256, 243/256];
        ColorLightRed=[243/256, 163/256, 152/256];

    end
    
    properties(Access=public)
        NameMovement = 'flexext'
        ROSConnected = 0
        
        Capture
        Interpolator
        
        DataInputShow
        DataInput
        
        DataAdjusted
        
        DataCommanded
        CommandedAngularVelocity

        DataOutput={};
        DataOutputCaptured={};
        DataReferenceCaptured;
        DataReference;

        
        %Repeatibility properties
        DataMin
        DataMean
        DataMax
        MeanError
        MinError
        MaxError
        MeanEucError
        MinEucError
        MaxEucError
        
        VelocityFactor
        
        SendTrajectoryMsg
        SendTrajectoryClient
        SendPositionMsg
        SendPositionClient
        SetGeneralVelocityClient
        SetGeneralVelocityMsg
        ConfigurationBlockRobotClient
        ConfigurationClient
        ConfigurationMsg
        ControlModeMsg
        
        DefaultCapTrajectoryList = 'flexext_P001_L_000000_0000_cap'% Description
        DefaultAdjTrajectoryList = 'flexext_P001_L_000000_0000_adj'% Description
    end
    
    properties(GetAccess=public, SetAccess=private)
        NameData
        NameDataInput
        NameDataCommanded
        CapturedDate
        
        InputAngularVelocity = 15;
        FlexionCaptured = 1; % 1 = flexion; 0 = extension
        OutputSmoothing = 1e-9;
        CommandedSmoothing = 1e-4;
        InputSmoothing = 1e-9;

    end

    methods
%% CONSTRUCTOR
        function obj = IIWAControl(ConnectROS)
            %addpath(genpath(fileparts(which(mfilename))));
            %savepath;
            if (ConnectROS==1)
                if (robotics.ros.internal.Global.isNodeActive)
                    rosshutdown;
                end
                if (isunix)
                    [~, result] = system('ifconfig');
                else
                    [~, result] = system('ipconfig');
                end
                dir = ['http://', obj.MasterIP, ':', obj.MasterPort];
                rosinit(dir, 'NodeName', 'Matlab');
                warning('off', 'MATLAB:MKDIR:DirectoryExists');
                obj.SetMaximumGeneralVelocity();
                obj.ROSConnected=1;
                obj.InitServiceClients();
                obj.Capture=Capture();
            else
                obj.ROSConnected=0;
            end
        end
%% PUBLIC METHODS
        % SET PRIVATE PROPERTIES FUNCTIONS -> Allow to change some
        % properties but just in specific ways
        function SetMovement(obj, name)
            obj.NameMovement=name;
        end
        function [IDPatient, Arm] = SetNameData(obj, name_data)
            obj.NameData=name_data;
            obj.NameDataCommanded=[obj.NameData, obj.CommandedIdentifier];
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
        function SetSmoothingParameters(obj, input, commanded, output)
            obj.InputSmoothing = input;
            obj.CommandedSmoothing = commanded;
            obj.OutputSmoothing = output;
        end
        function SetCapturedMovement(obj, movement)
            obj.FlexionCaptured = movement;
        end
        % ROBOT MOVEMENT FUNCTIONS
        function SendReferencePositionLeft(obj)
            if (obj.ROSConnected)
            	obj.SendJointPosition(obj.ReferencePositionLeft);
            end
        end
        function SendReferencePositionRight(obj)
            if (obj.ROSConnected)
            	obj.SendJointPosition(obj.ReferencePositionRight);
            end
        end
        function [IDPatient, Arm] = SendTrajectory(obj)
            [IDPatient, Arm] = obj.SetNameData(obj.NameData);
            if (obj.ROSConnected)
                obj.SendTrajectoryMsg.Name=obj.NameDataCommanded;
                obj.SendTrajectoryMsg.Folder='commanded';
                obj.SendTrajectoryClient.call(obj.SendTrajectoryMsg);
            else
                pause(5);
            end
        end
        function SendInitialPosition(obj)
           	if (obj.ROSConnected)
                if (~isempty(obj.DataCommanded))
                   	obj.SendJointPosition(obj.DataCommanded.q(:,1));
                else
                    ME=MException('IIWAControl:EmptyDataCommanded', 'DataCommanded est� vac�o');
                    throw(ME);
                end
            end
        end
        % CAPTURE FUNCTIONS
        function CaptureStart(obj)
            if (obj.ROSConnected)
                obj.Capture.Start();
                obj.FreeCartesianCoordinate('YZC');
            end
        end
        % CONFIGURATION MODE FUNCTIONS
        function SetGeneralVelocity(obj, v)
            if (obj.ROSConnected)
                obj.SetGeneralVelocityMsg.JointRelativeAcceleration=v;
                obj.SetGeneralVelocityMsg.JointRelativeVelocity=v;
                obj.SetGeneralVelocityMsg.OverrideJointAcceleration=v;
                obj.SetGeneralVelocityClient.call(obj.SetGeneralVelocityMsg, 'Timeout', 1);
            end
        end
        function FreeJoints(obj)
            if (obj.ROSConnected)
                obj.ConfigurationMsg.ControlMode=obj.ControlModeMsg.JOINTIMPEDANCE;
                obj.ConfigurationMsg.JointImpedance.JointDamping.A1=0.7;
                obj.ConfigurationMsg.JointImpedance.JointDamping.A2=0.7;
                obj.ConfigurationMsg.JointImpedance.JointDamping.A3=0.7;
                obj.ConfigurationMsg.JointImpedance.JointDamping.A4=0.7;
                obj.ConfigurationMsg.JointImpedance.JointDamping.A5=0.7;
                obj.ConfigurationMsg.JointImpedance.JointDamping.A6=0.7;
                obj.ConfigurationMsg.JointImpedance.JointDamping.A7=0.7;
                obj.ConfigurationMsg.JointImpedance.JointStiffness.A1=10;
                obj.ConfigurationMsg.JointImpedance.JointStiffness.A2=10;
                obj.ConfigurationMsg.JointImpedance.JointStiffness.A3=10;
                obj.ConfigurationMsg.JointImpedance.JointStiffness.A4=0;
                obj.ConfigurationMsg.JointImpedance.JointStiffness.A5=0;
                obj.ConfigurationMsg.JointImpedance.JointStiffness.A6=0;
                obj.ConfigurationMsg.JointImpedance.JointStiffness.A7=0;
                obj.ConfigurationClient.call(obj.ConfigurationMsg);
            end
        end
        function FreeCartesianCoordinate(obj, coord) %X, Y, Z, A, B, C
            if (obj.ROSConnected)
                damping=[0.7 0.7 0.7 0.7 0.7 0.7 0.7];
                stiffness=[1500 1500 1500 300 300 300 100];
                if (contains(coord, obj.NameCoord(1)))
                    stiffness(1)=1;
                end
                if (contains(coord, obj.NameCoord(2)))
                    stiffness(2)=1;
                end
                if (contains(coord, obj.NameCoord(3)))
                    stiffness(3)=1;
                end
                if (contains(coord, obj.NameCoord(4)))
                    stiffness(4)=1;
                end
                if (contains(coord, obj.NameCoord(5)))
                    stiffness(5)=1;
                end
                if (contains(coord, obj.NameCoord(6)))
                    stiffness(6)=1;
                end
                obj.ConfigurationMsg.ControlMode=obj.ControlModeMsg.CARTESIANIMPEDANCE;
                obj.ConfigurationMsg.CartesianImpedance.CartesianDamping.X=damping(1);
                obj.ConfigurationMsg.CartesianImpedance.CartesianDamping.Y=damping(2);
                obj.ConfigurationMsg.CartesianImpedance.CartesianDamping.Z=damping(3);
                obj.ConfigurationMsg.CartesianImpedance.CartesianDamping.A=damping(4);
                obj.ConfigurationMsg.CartesianImpedance.CartesianDamping.B=damping(5);
                obj.ConfigurationMsg.CartesianImpedance.CartesianDamping.C=damping(6);
                obj.ConfigurationMsg.CartesianImpedance.NullspaceDamping=damping(7);
                obj.ConfigurationMsg.CartesianImpedance.CartesianStiffness.X=stiffness(1);
                obj.ConfigurationMsg.CartesianImpedance.CartesianStiffness.Y=stiffness(2);
                obj.ConfigurationMsg.CartesianImpedance.CartesianStiffness.Z=stiffness(3);
                obj.ConfigurationMsg.CartesianImpedance.CartesianStiffness.A=stiffness(4);
                obj.ConfigurationMsg.CartesianImpedance.CartesianStiffness.B=stiffness(5);
                obj.ConfigurationMsg.CartesianImpedance.CartesianStiffness.C=stiffness(6);
                obj.ConfigurationMsg.CartesianImpedance.NullspaceStiffness=stiffness(7);
                obj.ConfigurationClient.call(obj.ConfigurationMsg);
            end
        end
        function BlockRobot(obj)
            if (obj.ROSConnected)
                try
                    obj.ConfigurationMsg.ControlMode=obj.ControlModeMsg.POSITIONCONTROL;
                    obj.ConfigurationClient.call(obj.ConfigurationMsg);
                    obj.ConfigurationClient.call(obj.ConfigurationMsg);
                catch
                    ME = MException('IIWAControl:ControlPositionError1', 'Mueva el robot a una posici�n m�s alejada de singularidades, resetee ROSSmartServo en el SmartPad e intentelo de nuevo');
                    throw(ME);
                end
            end    
        end
        % DATA INPUT FUNCTIONS
        function InputList = DataInputList(obj)
            if (obj.ROSConnected)
                InputList = obj.Capture.List(obj.InputIdentifier);
            else
                InputList = obj.DefaultCapTrajectoryList;
            end
        end
        function [IDPatient, Arm] = LoadInput(obj, name_input) 
            [IDPatient, Arm] = obj.SetNameData(name_input(1:strfind(name_input, obj.InputIdentifier)-1));
            %Load data input
            if (obj.ROSConnected)
                obj.DataInput = obj.Capture.Load(obj.NameDataInput, 'entrada');
            end
        end
        function SaveDataInput(obj,  id_patient, arm)
            obj.CapturedDate=datestr(now, 'yymmdd_HHMMSS');
            obj.BuildNameData(id_patient, arm); %This automatically changes also obj.NameDataInput
            if (obj.ROSConnected)
                obj.Capture.Stop();
                obj.DataInput=obj.CaptureSaveAndSmooth(obj.NameDataInput, 'entrada', 1, obj.InputSmoothing);
            else
                obj.DefaultCapTrajectoryList=[obj.DefaultCapTrajectoryList, {obj.NameDataInput}];
            end
        end
        % DATA ADJUSTED/COMMANDED FUNCTIONS
        function CommandedList = DataCommandedList(obj)
            if (obj.ROSConnected)
                CommandedList = obj.Capture.List(obj.CommandedIdentifier);
            else
                CommandedList = obj.DefaultAdjTrajectoryList;
            end
        end
        function [IDPatient, Arm] = LoadCommanded(obj, name_commanded) 
            [IDPatient, Arm] = obj.SetNameData(name_commanded(1:strfind(name_commanded, obj.CommandedIdentifier)-1));
            %Load data input
            if (obj.ROSConnected)
                obj.DataAdjusted = obj.Capture.Load(obj.NameDataCommanded, 'entrada');
            end
        end
        function [IDPatient, Arm] = BuildDataAdjusted(obj, name_input, bPlot, velocity)
            [IDPatient, Arm] = obj.SetNameData(name_input(1:strfind(name_input, obj.InputIdentifier)-1));
            if (obj.ROSConnected)
                data_resampled = change_tsample(obj.DataInput, obj.CommandedSampleTime); %TODO
                [obj.DataAdjusted, obj.InputAngularVelocity] = find_circle_and_mirror(data_resampled, obj.CommandedMiddlePause, obj.CommandedSmoothing, bPlot, obj.FlexionCaptured, 1, velocity); %TODO
                obj.Capture.New(obj.NameDataCommanded, 'entrada', obj.DataAdjusted); %TODO
            else
                obj.DefaultAdjTrajectoryList=[obj.DefaultAdjTrajectoryList, {obj.NameDataCommanded}];
            end
        end
        function ChangeVelocity(obj, velocity)
            obj.VelocityFactor=velocity;
            if (obj.ROSConnected)
                if (~isempty(obj.DataAdjusted))
                    obj.DataCommanded = change_velocity(obj.DataAdjusted, velocity);
                    obj.Capture.New(obj.NameDataCommanded, 'commanded', obj.DataCommanded);
                    if (~isempty(obj.DataInput))
                        data_input_withoutpauses = delete_pause_init_end(obj.DataInput);
                        obj.DataInputShow = change_velocity(data_input_withoutpauses, velocity);
                    else
                        ME=MException('IIWAControl:EmptyDataInput', 'DataInput est� vac�o');
                        throw(ME);
                    end
                else
                    ME=MException('IIWAControl:EmptyDataAdjusted', 'DataAdjusted est� vac�o');
                    throw(ME);
                end
            end
        end
        % DATA OUTPUT FUNCTIONS
        function SaveDataOutput(obj, i_trial)
            if (obj.ROSConnected)
                obj.DataOutputCaptured{i_trial} = obj.Capture.SaveAndLoad(obj.NameDataCommanded, 'salida');
            end
        end
        function SaveDataReference(obj)
            if (obj.ROSConnected)
                obj.DataReferenceCaptured = obj.Capture.SaveAndLoad(obj.NameDataCommanded, 'salida');
                obj.DataReference = obj.SmoothData(obj.DataReferenceCaptured, obj.VelocityFactor, obj.OutputSmoothing);
            end
        end
        function Trajectory = GetStruct(obj, i_trial)
            if (obj.ROSConnected)
                obj.DataOutput{i_trial} = obj.SmoothData(obj.DataOutputCaptured{i_trial}, obj.VelocityFactor, obj.OutputSmoothing);
                Trajectory.CapturedDate = obj.CapturedDate;
                Trajectory.Reference.Timestamps = obj.DataReference.t;
                Trajectory.Reference.JointTrajectory = obj.DataReference.q;
                Trajectory.Reference.CartesianTrajectory.Position = obj.DataReference.x.pos;
                Trajectory.Reference.CartesianTrajectory.Orientation = obj.DataReference.x.ori;
                Trajectory.Reference.JointVelocities = obj.DataReference.qdot;
                Trajectory.Reference.CartesianVelocities.Position = obj.DataReference.xdot.pos;
                Trajectory.Reference.CartesianVelocities.Orientation = obj.DataReference.xdot.ori;
                Trajectory.Reference.JointAccelerations = obj.DataReference.qdotdot;
                Trajectory.Reference.InterpolatorPolynomials.Breaks = obj.DataReference.pp.breaks;
                Trajectory.Reference.InterpolatorPolynomials.Coefficients.JointTrajectory = obj.DataReference.pp.coefs.q;
                Trajectory.Reference.InterpolatorPolynomials.Coefficients.JointVelocityTrajectory = obj.DataReference.pp.coefs.qdot;
                Trajectory.Reference.InterpolatorPolynomials.Coefficients.JointAccelerationTrajectory = obj.DataReference.pp.coefs.qdotdot;
                Trajectory.Reference.JointTorques.Timestamps = obj.DataReference.t_torque;
                Trajectory.Reference.JointTorques.Torques = obj.DataReference.q_torque;
                Trajectory.Trial.Timestamps = obj.DataOutput{i_trial}.t;
                Trajectory.Trial.JointTrajectory = obj.DataOutput{i_trial}.q;
                Trajectory.Trial.CartesianTrajectory.Position = obj.DataOutput{i_trial}.x.pos;
                Trajectory.Trial.CartesianTrajectory.Orientation = obj.DataOutput{i_trial}.x.ori;
                Trajectory.Trial.JointVelocities = obj.DataOutput{i_trial}.qdot;
                Trajectory.Trial.CartesianVelocities.Position = obj.DataOutput{i_trial}.xdot.pos;
                Trajectory.Trial.CartesianVelocities.Orientation = obj.DataOutput{i_trial}.xdot.ori;
                Trajectory.Trial.JointAccelerations = obj.DataOutput{i_trial}.qdotdot;
                Trajectory.Trial.InterpolatorPolynomials.Breaks = obj.DataOutput{i_trial}.pp.breaks;
                Trajectory.Trial.InterpolatorPolynomials.Coefficients.JointTrajectory = obj.DataOutput{i_trial}.pp.coefs.q;
                Trajectory.Trial.InterpolatorPolynomials.Coefficients.JointVelocityTrajectory = obj.DataOutput{i_trial}.pp.coefs.qdot;
                Trajectory.Trial.InterpolatorPolynomials.Coefficients.JointAccelerationTrajectory = obj.DataOutput{i_trial}.pp.coefs.qdotdot;
                Trajectory.Trial.JointTorques.Timestamps = obj.DataOutput{i_trial}.t_torque;
                Trajectory.Trial.JointTorques.Torques = obj.DataOutput{i_trial}.q_torque;
                Trajectory.DataCommanded = obj.DataCommanded;
            else
                Trajectory=[];
            end
        end
        % PLOT FUNCTIONS
        % Plot trajectories
        function PlotJointPositionTrajectories(obj, origin, arg, display)
            sgtitle(display, 'Posici�n articular');
            switch origin
                case 'results'
                    i_trial = arg;
                    if (obj.ROSConnected)
                        if (i_trial~=0)
                            obj.PlotJointPositionTrajectory(obj.DataInputShow, obj.ColorDataInput, 0, display);
                            obj.PlotJointPositionTrajectory(obj.DataCommanded, obj.ColorDataCommanded, 0, display);
                            ax = obj.PlotJointPositionTrajectory(obj.DataOutput{i_trial}, obj.ColorDataOutput, 1, display);
                            legend(ax(end), obj.FigureLegend);
                        else
                            obj.CalculateMinMeanMax(obj.DataOutput);
                            ax = obj.PlotJointPositionTrajectoryRepeatibility(obj.DataCommanded, display);  
                            f = get(ax(end), 'Children');
                            legend([f(1), f(2)], 'Comandada', 'Ejecutada')
                        end
                    end
                case 'database'
                    Trajectory = arg;
                    if(~isempty(Trajectory))
                        [trajectoryOutput, trajectoryCommanded] = IIWAControl.SetTrajectoryFromFile(Trajectory);
                        if max(size(trajectoryOutput))==1
                            obj.PlotJointPositionTrajectory(trajectoryCommanded, obj.ColorDataCommanded, 0, display);
                            ax = obj.PlotJointPositionTrajectory(trajectoryOutput, obj.ColorDataOutput, 1, display);
                            legend(ax(end), 'Comandada', 'Ejecutada');
                        else
                            obj.CalculateMinMeanMax(trajectoryOutput);
                            ax = obj.PlotJointPositionTrajectoryRepeatibility(trajectoryCommanded, display);  
                            f = get(ax(end), 'Children');
                            legend([f(1), f(2)], 'Comandada', 'Ejecutada')
                        end
                    else
                        ME = MException('IIWAControl:PlotJointPositionTrajectories', 'Empty Trajectory structure');
                        throw(ME);
                    end
                otherwise
                    ME=MException('IIWAControl:PlotJointPositionTrajectories', 'Wrong data origin specified. Must be "results" or "database"');
                    throw(ME);
            end
        end
        function PlotJointVelocityTrajectories(obj, origin, arg, display)
            sgtitle(display, 'Velocidad articular');
            switch origin
                case 'results'
                    i_trial = arg;
                    if (obj.ROSConnected)
                        if (i_trial~=0)
                            obj.PlotJointVelocityTrajectory(obj.DataInputShow, obj.ColorDataInput, 0, display);
                            obj.PlotJointVelocityTrajectory(obj.DataCommanded, obj.ColorDataCommanded, 0, display);
                            ax = obj.PlotJointVelocityTrajectory(obj.DataOutput{i_trial}, obj.ColorDataOutput, 1, display);
                            legend(ax(end), obj.FigureLegend);
                        else
                            obj.CalculateMinMeanMax(obj.DataOutput);
                            ax = obj.PlotJointVelocityTrajectoryRepeatibility(obj.DataCommanded, display);
                            f = get(ax(end), 'Children');
                            legend([f(1), f(2)], 'Comandada', 'Ejecutada')
                        end
                    end
                case 'database'
                    Trajectory = arg;
                    if (~isempty(Trajectory))
                        [trajectoryOutput, trajectoryCommanded] = IIWAControl.SetTrajectoryFromFile(Trajectory);
                        if max(size(trajectoryOutput))==1
                            obj.PlotJointVelocityTrajectory(trajectoryCommanded, obj.ColorDataCommanded, 0, display);
                            ax = obj.PlotJointVelocityTrajectory(trajectoryOutput, obj.ColorDataOutput, 1, display);
                            legend(ax(end), 'Comandada', 'Ejecutada')
                        else
                            obj.CalculateMinMeanMax(trajectoryOutput);
                            ax = obj.PlotJointVelocityTrajectoryRepeatibility(trajectoryCommanded, display);
                            f = get(ax(end), 'Children');
                            legend([f(1), f(2)], 'Comandada', 'Ejecutada')
                        end 
                    else
                        ME = MException('IIWAControl:PlotJointVelocityTrajectories', 'Empty Trajectory structure');
                        throw(ME);
                    end
                otherwise
                    ME=MException('IIWAControl:PlotJointVelocityTrajectories', 'Wrong data origin specified. Must be "results" or "database"');
                    throw(ME);
            end
        end
        function PlotJointAccelerationTrajectories(obj, origin, arg, display)
            sgtitle(display, 'Aceleraci�n articular');
            switch origin
                case 'results'
                    i_trial = arg;
                    if (obj.ROSConnected)
                        if (i_trial~=0)
                            obj.PlotJointAccelerationTrajectory(obj.DataInputShow, obj.ColorDataInput, 0, display);
                            obj.PlotJointAccelerationTrajectory(obj.DataCommanded, obj.ColorDataCommanded, 0, display);
                            ax = obj.PlotJointAccelerationTrajectory(obj.DataOutput{i_trial}, obj.ColorDataOutput, 1, display);
                            legend(ax(end), obj.FigureLegend);
                        else 
                            obj.CalculateMinMeanMax(obj.DataOutput);
                            ax = obj.PlotJointAccelerationTrajectoryRepeatibility(obj.DataCommanded, display);
                            f = get(ax(end), 'Children');
                            legend([f(1), f(2)], 'Comandada', 'Ejecutada')
                        end
                    end
                case 'database'
                    Trajectory = arg;
                    if (~isempty(Trajectory))
                        [trajectoryOutput, trajectoryCommanded] = IIWAControl.SetTrajectoryFromFile(Trajectory);
                        if max(size(trajectoryOutput))==1
                            obj.PlotJointAccelerationTrajectory(trajectoryCommanded, obj.ColorDataCommanded, 0, display);
                            ax = obj.PlotJointAccelerationTrajectory(trajectoryOutput, obj.ColorDataOutput, 1, display);
                            legend(ax(end), 'Comandada', 'Ejecutada')
                        else 
                            obj.CalculateMinMeanMax(trajectoryOutput);
                            ax = obj.PlotJointAccelerationTrajectoryRepeatibility(trajectoryCommanded, display);
                            f = get(ax(end), 'Children');
                            legend([f(1), f(2)], 'Comandada', 'Ejecutada')
                        end
                    else
                        ME = MException('IIWAControl:PlotJointAccelerationTrajectories', 'Empty Trajectory structure');
                        throw(ME);
                    end
                otherwise
                    ME=MException('IIWAControl:PlotJointAccelerationTrajectories', 'Wrong data origin specified. Must be "results" or "database"');
                    throw(ME);
            end
        end
        function PlotCartesianPositionTrajectories(obj, origin, arg, display)
            sgtitle(display, 'Posici�n cartesiana');
            switch origin
                case 'results'
                    i_trial = arg;
                    if (obj.ROSConnected)
                        if (i_trial~=0)
                            obj.PlotCartesianPositionTrajectory(obj.DataInputShow, obj.ColorDataInput, display);
                            obj.PlotCartesianPositionTrajectory(obj.DataCommanded, obj.ColorDataCommanded, display);
                            ax = obj.PlotCartesianPositionTrajectory(obj.DataOutput{i_trial}, obj.ColorDataOutput, display);
                            legend(ax(end), obj.FigureLegend);
                        else
                            obj.CalculateMinMeanMax(obj.DataOutput);
                            ax = obj.PlotCartesianPositionTrajectoryRepeatibility(obj.DataCommanded, display);
                            f = get(ax(end), 'Children');
                            legend([f(1), f(2)], 'Comandada', 'Ejecutada')
                        end
                    end
                case 'database'
                    Trajectory = arg;
                    if (~isempty(Trajectory))
                        [trajectoryOutput, trajectoryCommanded] = IIWAControl.SetTrajectoryFromFile(Trajectory);
                        if max(size(trajectoryOutput))==1
                            obj.PlotCartesianPositionTrajectory(trajectoryCommanded, obj.ColorDataCommanded, display);
                            ax = obj.PlotCartesianPositionTrajectory(trajectoryOutput, obj.ColorDataOutput, display);
                            legend(ax(end), 'Comandada', 'Ejecutada')
                        else
                            obj.CalculateMinMeanMax(trajectoryOutput);
                            ax = obj.PlotCartesianPositionTrajectoryRepeatibility(trajectoryCommanded, display);
                            f = get(ax(end), 'Children');
                            legend([f(1), f(2)], 'Comandada', 'Ejecutada')
                        end
                    else
                        ME = MException('IIWAControl:PlotCartesianPositionTrajectories', 'Empty Trajectory structure');
                        throw(ME);
                    end
                case 'adjust'
                    if(obj.ROSConnected)
                        display.WindowState = 'maximized';
                        display.Name = 'Posici�n cartesiana';
                        hold on;
                        obj.PlotCartesianPositionTrajectory(obj.DataInputShow, obj.ColorDataInput, display);
                        obj.PlotCartesianPositionTrajectory(obj.DataCommanded, obj.ColorDataCommanded, display);
                        legend('Grabada', 'Ajustada')
                    end
                otherwise
                    ME=MException('IIWAControl:PlotCartesianPositionTrajectories', 'Wrong data origin specified. Must be "results", "database" or "adjust"');
                    throw(ME);
            end
        end
        function Plot3DCartesianPositionTrajectories(obj, origin, arg, display)
            sgtitle(display, 'Trayectoria cartesiana en 3D');
            switch origin
                case 'results'
                    i_trial = arg;
                    if (obj.ROSConnected)
                        if (i_trial~=0)
                            if(strcmp(display.Type, 'uitab'))
                                ax = subplot(1,1,1,'Parent', display);%uiaxes(display, 'Position', display.InnerPosition*0.92);
                            else
                                display.Name = 'Posici�n cartesiana 3D';
                                display.WindowState = 'maximized';
                                ax = axes;
                            end
                            hold(ax, 'on');
                            obj.Plot3DCartesianPositionTrajectory(obj.DataInputShow,obj.ColorDataInput, ax);
                            obj.Plot3DCartesianPositionTrajectory(obj.DataCommanded,obj.ColorDataCommanded, ax);
                            obj.Plot3DCartesianPositionTrajectory(obj.DataOutput{i_trial},obj.ColorDataOutput, ax);
                            view(ax, [-120, 30]);
                            axis(ax, 'equal');
                            legend(ax, {'Grabada', 'Comandada', 'Ejecutada'}, 'Location', 'northwest');
                        else
                            obj.CalculateMinMeanMax(obj.DataOutput);
                            if(strcmp(display.Type, 'uitab'))
                                ax = subplot(1,1,1,'Parent', display);%uiaxes(display, 'Position', display.InnerPosition*0.92);
                            else
                                display.Name = 'Posici�n cartesiana 3D';
                                display.WindowState = 'maximized';
                                ax = axes;
                            end
                            hold(ax, 'on');
                            for i = 1:max(size(obj.DataOutput))
                                obj.Plot3DCartesianPositionTrajectory(obj.DataOutput{i},obj.ColorDataOutput, ax);
                            end
                            obj.Plot3DCartesianPositionTrajectory(obj.DataCommanded, obj.ColorDataCommanded, ax);
                            view(ax, [-120, 30]);
                            axis(ax, 'equal');
                            f = get(ax, 'Children');
                            legend([f(1), f(2)], {'Comandada', 'Ejecutada'}, 'Location', 'northwest')
                        end
                    end
                case 'database'
                    Trajectory = arg;
                    if(~isempty(Trajectory))
                        [trajectoryOutput, trajectoryCommanded] = IIWAControl.SetTrajectoryFromFile(Trajectory);
                        if max(size(trajectoryOutput))==1
                            if(strcmp(display.Type, 'uitab'))
                                ax = subplot(1,1,1,'Parent', display);%uiaxes(display, 'Position', display.InnerPosition*0.92);
                            else
                                display.Name = 'Posici�n cartesiana 3D';
                                display.WindowState = 'maximized';
                                ax = axes;
                            end
                            hold(ax, 'on')
                            obj.Plot3DCartesianPositionTrajectory(trajectoryCommanded,obj.ColorDataCommanded, ax);
                            obj.Plot3DCartesianPositionTrajectory(trajectoryOutput,obj.ColorDataOutput, ax);
                            view(ax, [-120, 30]);
                            axis(ax, 'equal');
                            legend(ax, {'Comandada', 'Ejecutada'}, 'Location', 'northwest')
                        else
                            obj.CalculateMinMeanMax(trajectoryOutput);
                            if(strcmp(display.Type, 'uitab'))
                                ax = subplot(1,1,1,'Parent', display);%uiaxes(display, 'Position', display.InnerPosition*0.92);
                            else
                                display.Name = 'Posici�n cartesiana 3D';
                                display.WindowState = 'maximized';
                                ax = axes;
                            end
                            hold(ax, 'on');
                            for i = 1:max(size(trajectoryOutput))
                                obj.Plot3DCartesianPositionTrajectory(trajectoryOutput{i},obj.ColorDataOutput, ax);
                            end
                            obj.Plot3DCartesianPositionTrajectory(trajectoryCommanded, obj.ColorDataCommanded, ax);
                            view(ax, [-120, 30]);
                            axis(ax, 'equal');
                            f = get(ax, 'Children');
                            legend([f(1), f(2)], {'Comandada', 'Ejecutada'}, 'Location', 'northwest');
                        end
                    else
                        ME = MException('IIWAControl:Plot3DCartesianPositionTrajectories', 'Empty Trajectory structure');
                        throw(ME);
                    end
                case 'adjust'
                    if (obj.ROSConnected)
                        display.WindowState = 'maximized';
                        display.Name = 'Posici�n cartesiana 3D';
                        hold on;
                        ax = axes;
                        obj.Plot3DCartesianPositionTrajectory(obj.DataInputShow,obj.ColorDataInput, ax);
                        obj.Plot3DCartesianPositionTrajectory(obj.DataCommanded,obj.ColorDataCommanded, ax);
                        view([-120, 30]);
                        axis equal;
                        legend('Grabada', 'Comandada');
                    end
                otherwise
                    ME=MException('IIWAControl:Plot3DCartesianPositionTrajectories', 'Wrong data origin specified. Must be "results", "database" or "adjust"');
                    throw(ME);
            end
        end
        function PlotCartesianVelocityTrajectories(obj, origin, arg, display)
            sgtitle(display, 'Velocidad cartesiana');
            switch origin
                case 'results'
                    i_trial = arg;
                    if (obj.ROSConnected)
                        if (i_trial~=0)
%                             figure('WindowState', 'maximized', 'Name', 'Velocidad cartesiana');
%                             hold on;
                            obj.PlotCartesianVelocityTrajectory(obj.DataInputShow, obj.ColorDataInput, display);
                            obj.PlotCartesianVelocityTrajectory(obj.DataCommanded, obj.ColorDataCommanded, display);
                            ax = obj.PlotCartesianVelocityTrajectory(obj.DataOutput{i_trial}, obj.ColorDataOutput, display);
                            legend(ax(end), 'Grabada', 'Comandada', 'Ejecutada');
                        else
                            obj.CalculateMinMeanMax(obj.DataOutput);
                            ax = obj.PlotCartesianVelocityTrajectoryRepeatibility(obj.DataCommanded, display);
                            f = get(ax(end), 'Children');
                            legend(ax(end), [f(1), f(2)], 'Comandada', 'Ejecutada')
                        end
                    end
                case 'database'
                    Trajectory = arg;
                    if(~isempty(Trajectory))
                        [trajectoryOutput, trajectoryCommanded] = IIWAControl.SetTrajectoryFromFile(Trajectory);
                        if max(size(trajectoryOutput))==1
                            obj.PlotCartesianVelocityTrajectory(trajectoryCommanded, obj.ColorDataCommanded, display);
                            ax = obj.PlotCartesianVelocityTrajectory(trajectoryOutput, obj.ColorDataOutput, display);
                            legend(ax(end), 'Comandada', 'Ejecutada')
                        else
                            obj.CalculateMinMeanMax(trajectoryOutput);
                            ax = obj.PlotCartesianVelocityTrajectoryRepeatibility(trajectoryCommanded, display);
                            f = get(ax(end), 'Children');
                            legend([f(1), f(2)], 'Comandada', 'Ejecutada')
                        end
                    else
                        ME = MException('IIWAControl:PlotCartesianVelocityTrajectories', 'Empty Trajectory structure');
                        throw(ME);
                    end
                case 'adjust'
                    if(obj.ROSConnected)
                        display.WindowState = 'maximized';
                        display.Name = 'Velocidad cartesiana';
                        hold on;
                        obj.PlotCartesianVelocityTrajectory(obj.DataInputShow, obj.ColorDataInput, display);
                        obj.PlotCartesianVelocityTrajectory(obj.DataCommanded, obj.ColorDataCommanded, display);
                        legend('Grabada', 'Ajustada')
                    end
                otherwise
                    ME=MException('IIWAControl:PlotCartesianVelocityTrajectories', 'Wrong data origin specified. Must be "results", "database" or "adjust"');
                    throw(ME);
            end
        end
        % Plot error (dataCommanded - dataOutput) or (dataInput - dataCommanded)
        function PlotJointPositionError(obj, origin, arg, display)
            switch origin
                case 'results'
                    i_trial = arg;
                    if (obj.ROSConnected)
                        if (i_trial~=0)
                            obj.PlotJointPositionError_(obj.DataCommanded, obj.DataOutput{i_trial}, display);
                        else
                            obj.CalculateMinMeanMax(obj.DataOutput);
                            obj.CalculateMinMeanMaxJointError(obj.DataCommanded, obj.DataOutput);
                            sgtitle(display, 'Error de repetibilidad de posici�n articular ejecutada frente a comandada');
                            obj.PlotJointPositionRepeatibility(display); 
                        end
                    end
                case 'database'
                    Trajectory = arg;
                    if(~isempty(Trajectory))
                        [trajectoryOutput, trajectoryCommanded] = IIWAControl.SetTrajectoryFromFile(Trajectory);
                        if max(size(trajectoryOutput))==1
                            obj.PlotJointPositionError_(trajectoryCommanded, trajectoryOutput, display);
                        else
                            obj.CalculateMinMeanMax(trajectoryOutput);
                            obj.CalculateMinMeanMaxJointError(trajectoryCommanded, trajectoryOutput);
                            sgtitle(display, 'Error de repetibilidad de posici�n articular ejecutada frente a comandada');
                            obj.PlotJointPositionRepeatibility(display);
                        end
                    else
                        ME = MException('IIWAControl:PlotJointPositionError', 'Empty Trajectory structure');
                        throw(ME);
                    end                        
                otherwise
                    ME=MException('IIWAControl:PlotJointPositionError', 'Wrong data origin specified. Must be "results" or "database"');
                    throw(ME);
            end
        end
        function PlotJointVelocityError(obj, origin, arg, display)
            switch origin
                case 'results'
                    i_trial = arg;
                    if (obj.ROSConnected)
                        if (i_trial~=0)
                            obj.PlotJointVelocityError_(obj.DataCommanded, obj.DataOutput{i_trial}, display);
                        else
                            obj.CalculateMinMeanMax(obj.DataOutput);
                            obj.CalculateMinMeanMaxJointError(obj.DataCommanded, obj.DataOutput);
                            sgtitle(display, 'Error de repetibilidad de velocidad articular ejecutada frente a comandada');
                            obj.PlotJointVelocityRepeatibility(display);
                        end    
                    end
                case 'database'
                    Trajectory = arg;
                    if(~isempty(Trajectory))
                        [trajectoryOutput, trajectoryCommanded] = IIWAControl.SetTrajectoryFromFile(Trajectory);
                        if max(size(trajectoryOutput))==1
                            obj.PlotJointVelocityError_(trajectoryCommanded, trajectoryOutput, display);
                        else
                            obj.CalculateMinMeanMax(trajectoryOutput);
                            obj.CalculateMinMeanMaxJointError(trajectoryCommanded, trajectoryOutput);
                            sgtitle(display, 'Error de repetibilidad de velocidad articular ejecutada frente a comandada');
                            obj.PlotJointVelocityRepeatibility(display);
                        end
                    else
                        ME = MException('IIWAControl:PlotJointVelocityError', 'Empty Trajectory structure');
                        throw(ME);
                    end
                otherwise
                    ME=MException('IIWAControl:PlotJointVelocityError', 'Wrong data origin specified. Must be "results" or "database"');
                    throw(ME);
            end
        end
        function PlotJointAccelerationError(obj, origin, arg, display)
            switch origin
                case 'results'
                    i_trial = arg;
                    if (obj.ROSConnected)
                        if (i_trial~=0)
                            obj.PlotJointAccelerationError_(obj.DataCommanded, obj.DataOutput{i_trial}, display);
                        else
                            obj.CalculateMinMeanMax(obj.DataOutput);
                            obj.CalculateMinMeanMaxJointError(obj.DataCommanded, obj.DataOutput);
                            sgtitle(display, 'Error de repetibilidad de aceleraci�n articular ejecutada frente a comandada');
                            obj.PlotJointAccelerationRepeatibility(display); 
                        end    
                    end
                case 'database'
                    Trajectory = arg;
                    if(~isempty(Trajectory))
                        [trajectoryOutput, trajectoryCommanded] = IIWAControl.SetTrajectoryFromFile(Trajectory);
                        if max(size(trajectoryOutput))==1
                            obj.PlotJointAccelerationError_(trajectoryCommanded, trajectoryOutput, display);
                        else
                            obj.CalculateMinMeanMax(trajectoryOutput);
                            obj.CalculateMinMeanMaxJointError(trajectoryCommanded, trajectoryOutput);
                            sgtitle(display, 'Error de repetibilidad de aceleraci�n articular ejecutada frente a comandada');
                            obj.PlotJointAccelerationRepeatibility(display);
                        end
                    else
                        ME = MException('IIWAControl:PlotJointAccelerationError', 'Empty Trajectory structure');
                        throw(ME);
                    end
                otherwise
                    ME=MException('IIWAControl:PlotJointAccelerationError', 'Wrong data origin specified. Must be "results" or "database"');
                    throw(ME);
            end
        end
        function PlotCartesianPositionError(obj, origin, arg, display)
            switch origin
                case 'results'
                    i_trial = arg;
                    if (obj.ROSConnected)
                        if (i_trial~=0)
                            obj.PlotCartesianPositionError_(obj.DataCommanded, obj.DataOutput{i_trial}, display);
                        else
                            obj.CalculateMinMeanMax(obj.DataOutput);
                            obj.CalculateMinMeanMaxCartesianError(obj.DataCommanded, obj.DataOutput);
                            sgtitle(display, 'Error de repetibilidad de posici�n cartesiana ejecutada frente a comandada');
                            obj.PlotCartesianPositionRepeatibility(display); 
                        end
                    end
                case 'database'
                    Trajectory = arg;
                    if(~isempty(Trajectory))
                        [trajectoryOutput, trajectoryCommanded] = IIWAControl.SetTrajectoryFromFile(Trajectory);
                        if max(size(trajectoryOutput))==1
                            obj.PlotCartesianPositionError_(trajectoryCommanded, trajectoryOutput, display);
                        else
                            obj.CalculateMinMeanMax(trajectoryOutput);
                            obj.CalculateMinMeanMaxCartesianError(trajectoryCommanded, trajectoryOutput);
                            sgtitle(display, 'Error de repetibilidad de posici�n cartesiana ejecutada frente a comandada');
                            obj.PlotCartesianPositionRepeatibility(display);
                        end
                    else
                        ME = MException('IIWAControl:PlotCartesianPositionError', 'Empty Trajectory structure');
                        throw(ME);
                    end
                case 'adjust'
                    if(obj.ROSConnected)
                        display.WindowState = 'maximized';
                        obj.PlotCartesianPositionError_(obj.DataInput, obj.DataCommanded, display);
                    end
                otherwise
                    ME=MException('IIWAControl:PlotCartesianPositionError', 'Wrong data origin specified. Must be "results", "database" or "adjust"');
                    throw(ME);
            end
        end
        function PlotCartesianVelocityError(obj, origin, arg, display)
            switch origin
                case 'results'
                    i_trial = arg;
                    if (obj.ROSConnected)
                        if (i_trial~=0)
                            obj.PlotCartesianVelocityError_(obj.DataCommanded, obj.DataOutput{i_trial}, display);
                        else
                            obj.CalculateMinMeanMax(obj.DataOutput);
                            obj.CalculateMinMeanMaxCartesianError(obj.DataCommanded, obj.DataOutput);
                            sgtitle(display, 'Error de repetibilidad de velocidad cartesiana ejecutada frente a comandada');
                            obj.PlotCartesianVelocityRepeatibility(display); 
                        end
                    end
                case 'database'
                    Trajectory = arg;
                    if(~isempty(Trajectory))
                        [trajectoryOutput, trajectoryCommanded] = IIWAControl.SetTrajectoryFromFile(Trajectory);
                        if max(size(trajectoryOutput))==1
                            obj.PlotCartesianVelocityError_(trajectoryCommanded, trajectoryOutput, display);
                        else
                            obj.CalculateMinMeanMax(trajectoryOutput);
                            obj.CalculateMinMeanMaxCartesianError(trajectoryCommanded, trajectoryOutput);
                            sgtitle(display, 'Error de repetibilidad de velocidad cartesiana ejecutada frente a comandada');
                            obj.PlotCartesianVelocityRepeatibility(display);
                        end
                    else
                        ME = MException('IIWAControl:PlotCartesianVelocityError', 'Empty Trajectory structure');
                        throw(ME);
                    end
                case 'adjust'
                    if(obj.ROSConnected)
                        display.WindowState = 'maximized';
                        obj.PlotCartesianVelocityError_(obj.DataInput, obj.DataCommanded, display);
                    end
                otherwise
                    ME=MException('IIWAControl:PlotCartesianVelocityError', 'Wrong data origin specified. Must be "results", "database" or "adjust"');
                    throw(ME);
            end
        end
        % Plot repeatibility errors (dataOutputMean - dataOutput) -> Use only to compare multiple trials 
        function PlotJointPositionRepeatibilityError(obj, origin, display, varargin)
            switch origin
                case 'results'
                    if(obj.ROSConnected)
                        meanOutput = get_mean(obj.DataOutput);
                        obj.CalculateMinMeanMaxJointError(meanOutput, obj.DataOutput);
                        sgtitle(display, 'Repetibilidad del error de posici�n articular ejecutada');
                        obj.PlotJointPositionRepeatibility(display);
                    end
                case 'database'
                    if max(size(varargin))==1
                        Trajectory = varargin{1};
                        if(~isempty(Trajectory))
                            if max(size(Trajectory))>1
                                [trajectoryOutput, ~] = IIWAControl.SetTrajectoryFromFile(Trajectory);
                                meanOutput = get_mean(trajectoryOutput);
                                obj.CalculateMinMeanMaxJointError(meanOutput, trajectoryOutput);
                                sgtitle(display, 'Repetibilidad del error de posici�n articular ejecutada');
                                obj.PlotJointPositionRepeatibility(display);
                            else
                                ME=MException('IIWAControl:PlotJointPositionRepeatibilityError', 'Number of trials insufficient. Must be greater than 1');
                                throw(ME);
                            end
                        else
                            ME = MException('IIWAControl:PlotJointPositionRepeatibilityError', 'Empty Trajectory structure');
                            throw(ME);
                        end
                    else
                        ME=MException('IIWAControl:PlotJointPositionRepeatibilityError', 'Not enough input arguments');
                        throw(ME);
                    end
                otherwise
                    ME=MException('IIWAControl:PlotJointPositionRepeatibilityError', 'Wrong data origin specified. Must be "results" or "database"');
                    throw(ME);
            end
        end
        function PlotJointVelocityRepeatibilityError(obj, origin, display, varargin)
            switch origin
                case 'results'
                    if(obj.ROSConnected)
                        meanOutput = get_mean(obj.DataOutput);
                        obj.CalculateMinMeanMaxJointError(meanOutput, obj.DataOutput);
                        sgtitle(display, 'Repetibilidad del error de velocidad articular ejecutada');
                        obj.PlotJointVelocityRepeatibility(display);
                    end
                case 'database'
                    if max(size(varargin))==1
                        Trajectory = varargin{1};
                        if(~isempty(Trajectory))
                            if max(size(Trajectory))>1
                                [trajectoryOutput, ~] = IIWAControl.SetTrajectoryFromFile(Trajectory);
                                meanOutput = get_mean(trajectoryOutput);
                                obj.CalculateMinMeanMaxJointError(meanOutput, trajectoryOutput);
                                sgtitle(display, 'Repetibilidad del error de velocidad articular ejecutada');
                                obj.PlotJointVelocityRepeatibility(display);
                            else
                                ME=MException('IIWAControl:PlotJointVelocityRepeatibilityError', 'Number of trials insufficient. Must be greater than 1');
                                throw(ME);
                            end
                        else
                            ME = MException('IIWAControl:PlotJointVelocityRepeatibilityError', 'Empty Trajectory structure');
                            throw(ME);
                        end
                    else
                        ME=MException('IIWAControl:PlotJointVelocityRepeatibilityError', 'Not enough input arguments');
                        throw(ME);
                    end
                otherwise
                    ME=MException('IIWAControl:PlotJointVelocityRepeatibilityError', 'Wrong data origin specified. Must be "results" or "database"');
                    throw(ME);
            end
        end
        function PlotJointAccelerationRepeatibilityError(obj, origin, display, varargin)
            switch origin
                case 'results'
                    if(obj.ROSConnected)
                        meanOutput = get_mean(obj.DataOutput);
                        obj.CalculateMinMeanMaxJointError(meanOutput, obj.DataOutput);
                        sgtitle(display, 'Repetibilidad del error de aceleraci�n articular ejecutada');
                        obj.PlotJointAccelerationRepeatibility(display);
                    end
                case 'database'
                    if max(size(varargin))==1
                        Trajectory = varargin{1};
                        if(~isempty(Trajectory))
                            if max(size(Trajectory))>1
                                [trajectoryOutput, ~] = IIWAControl.SetTrajectoryFromFile(Trajectory);
                                meanOutput = get_mean(trajectoryOutput);
                                obj.CalculateMinMeanMaxJointError(meanOutput, trajectoryOutput);
                                sgtitle(display, 'Repetibilidad del error de aceleraci�n articular ejecutada');
                                obj.PlotJointAccelerationRepeatibility(display);
                            else
                                ME=MException('IIWAControl:PlotJointAccelerationRepeatibilityError', 'Number of trials insufficient. Must be greater than 1');
                                throw(ME);
                            end
                        else
                            ME = MException('IIWAControl:PlotJointAccelerationRepeatibilityError', 'Empty Trajectory structure');
                            throw(ME);
                        end
                    else
                        ME=MException('IIWAControl:PlotJointAccelerationRepeatibilityError', 'Not enough input arguments.');
                        throw(ME);
                    end
                otherwise
                    ME=MException('IIWAControl:PlotJointAccelerationRepeatibilityError', 'Wrong data origin specified. Must be "results" or "database"');
                    throw(ME);
            end
        end
        function PlotCartesianPositionRepeatibilityError(obj, origin, display, varargin)
            switch origin
                case 'results'
                    if(obj.ROSConnected)
                        meanOutput = get_mean(obj.DataOutput);
                        obj.CalculateMinMeanMaxCartesianError(meanOutput, obj.DataOutput);
                        sgtitle(display, 'Repetibilidad del error de posici�n cartesiana ejecutada');
                        obj.PlotCartesianPositionRepeatibility(display);
                    end
                case 'database'
                    if max(size(varargin))==1
                        Trajectory = varargin{1};
                        if(~isempty(Trajectory))
                            if max(size(Trajectory))>1
                                [trajectoryOutput, ~] = IIWAControl.SetTrajectoryFromFile(Trajectory);
                                meanOutput = get_mean(trajectoryOutput);
                                obj.CalculateMinMeanMaxCartesianError(meanOutput, trajectoryOutput);
                                sgtitle(display, 'Repetibilidad del error de posici�n cartesiana ejecutada');
                                obj.PlotCartesianPositionRepeatibility(display);
                            else
                                ME=MException('IIWAControl:PlotCartesianPositionRepeatibilityError', 'Number of trials insufficient. Must be greater than 1');
                                throw(ME);
                            end
                        else
                            ME = MException('IIWAControl:PlotCartesianPositionRepeatibilityError', 'Empty Trajectory structure');
                            throw(ME);
                        end
                    else
                        ME=MException('IIWAControl:PlotCartesianPositionRepeatibilityError', 'Not enough input arguments');
                        throw(ME);
                    end
                otherwise
                    ME=MException('IIWAControl:PlotCartesianPositionRepeatibilityError', 'Wrong data origin specified. Must be "results" or "database"');
                    throw(ME);
            end
        end
        function PlotCartesianVelocityRepeatibilityError(obj, origin, display, varargin)
            switch origin
                case 'results'
                    if(obj.ROSConnected)
                        meanOutput = get_mean(obj.DataOutput);
                        obj.CalculateMinMeanMaxCartesianError(meanOutput, obj.DataOutput);
                        sgtitle(display, 'Repetibilidad del error de velocidad cartesiana ejecutada');
                        obj.PlotCartesianVelocityRepeatibility(display);
                    end
                case 'database'
                    if max(size(varargin))==1
                        Trajectory = varargin{1};
                        if(~isempty(Trajectory))
                            if max(size(Trajectory))>1
                                [trajectoryOutput, ~] = IIWAControl.SetTrajectoryFromFile(Trajectory);
                                meanOutput = get_mean(trajectoryOutput);
                                obj.CalculateMinMeanMaxCartesianError(meanOutput, trajectoryOutput);
                                sgtitle(display, 'Repetibilidad del error de velocidad cartesiana ejecutada');
                                obj.PlotCartesianVelocityRepeatibility(display);
                            else
                                ME=MException('IIWAControl:PlotCartesianVelocityRepeatibilityError', 'Number of trials insufficient. Must be greater than 1');
                                throw(ME);
                            end
                        else
                            ME = MException('IIWAControl:PlotCartesianVelocityRepeatibilityError', 'Empty Trajectory structure');
                            throw(ME);
                        end
                    else
                        ME=MException('IIWAControl:PlotCartesianVelocityRepeatibilityError', 'Not enough input arguments');
                        throw(ME);
                    end
                otherwise
                    ME=MException('IIWAControl:PlotCartesianVelocityRepeatibilityError', 'Wrong data origin specified. Must be "results" or "database"');
                    throw(ME);
            end
        end 
    end
%% PRIVATE METHODS
    methods(Access = private)
        function BuildNameData(obj, id_patient, arm)
            obj.NameData=strcat(obj.NameMovement, '_P', num2str(id_patient, '%03.f'), '_', arm, '_', obj.CapturedDate, '_');
            obj.NameDataCommanded=[obj.NameData, obj.CommandedIdentifier];
            obj.NameDataInput=[obj.NameData, obj.InputIdentifier];
        end
        function InitServiceClients(obj)
            if (obj.ROSConnected)
                [obj.SendTrajectoryClient, obj.SendTrajectoryMsg]=rossvcclient('roboespas/movement/send_joint_velocity_trajectory');
                [obj.SendPositionClient, obj.SendPositionMsg]=rossvcclient('roboespas/movement/send_joint_position');
                [obj.ConfigurationClient, obj.ConfigurationMsg]=rossvcclient('iiwa/configuration/configureSmartServo');
                [obj.SetGeneralVelocityClient, obj.SetGeneralVelocityMsg] = rossvcclient('iiwa/configuration/pathParameters');
                obj.ControlModeMsg=rosmessage('iiwa_msgs/ControlMode');
            end
        end
        % CONFIGURATION FUNCTIONS
        function SetMaximumGeneralVelocity(obj)
            if (obj.ROSConnected)
                obj.SetGeneralVelocityMsg.JointRelativeAcceleration=1;
                obj.SetGeneralVelocityMsg.JointRelativeVelocity=1;
                obj.SetGeneralVelocityMsg.OverrideJointAcceleration=1;
                obj.SetGeneralVelocityClient.call(obj.SetGeneralVelocityMsg, 'Timeout', 1);
            end
        end
        % CAPTURE FUNCTIONS
        function DataSmooth = CaptureSaveAndSmooth(obj, name, folder, velocity, smooth)
            if (obj.ROSConnected)
            	DataCaptured = obj.Capture.SaveAndLoad(name, folder);
                DataSmooth = obj.SmoothData(DataCaptured, velocity, smooth);
                obj.Capture.New(name, folder, DataSmooth);
            end
        end
        % ROBOT MOVEMENT
        function SendJointPosition(obj, JointPosition)
            if (obj.ROSConnected)
                obj.SendPositionMsg.A1=JointPosition(1);
                obj.SendPositionMsg.A2=JointPosition(2);
                obj.SendPositionMsg.A3=JointPosition(3);
                obj.SendPositionMsg.A4=JointPosition(4);
                obj.SendPositionMsg.A5=JointPosition(5);
                obj.SendPositionMsg.A6=JointPosition(6);
                obj.SendPositionMsg.A7=JointPosition(7);   
                obj.SendPositionClient.call(obj.SendPositionMsg);
            end
        end   
        % PLOT
        function CalculateMinMeanMax(obj, DataOutput)
            DataUsed = DataOutput;
            nsamples=20;
            for i=1:size(DataOutput,2)
                DataUsed{i}.q = zeros(size(DataOutput{i}.q,1), floor(size(DataOutput{i}.q,2)/nsamples)+3); % +3 to include the last samples that don't necessarily form a pack and to include the first and last samples as they are
                DataUsed{i}.qdot = zeros(size(DataOutput{i}.qdot,1), floor(size(DataOutput{i}.qdot,2)/nsamples)+3);
                DataUsed{i}.qdotdot = zeros(size(DataOutput{i}.qdotdot,1), floor(size(DataOutput{i}.qdotdot,2)/nsamples)+3);
                DataUsed{i}.x.pos = zeros(size(DataOutput{i}.x.pos,1), floor(size(DataOutput{i}.x.pos,2)/nsamples)+3);
                DataUsed{i}.x.ori = zeros(size(DataOutput{i}.x.ori,1), floor(size(DataOutput{i}.x.ori,2)/nsamples)+3);
                DataUsed{i}.xdot.pos = zeros(size(DataOutput{i}.xdot.pos,1), floor(size(DataOutput{i}.xdot.pos,2)/nsamples)+3);
                DataUsed{i}.xdot.ori = zeros(size(DataOutput{i}.xdot.ori,1), floor(size(DataOutput{i}.xdot.ori,2)/nsamples)+3);
                DataUsed{i}.t = zeros(size(DataOutput{i}.t,1), floor(size(DataOutput{i}.t,2)/nsamples)+3);

                for j = 1:nsamples:(size(DataOutput{i}.q,2)-nsamples)
                    DataUsed{i}.q(:,ceil(j/nsamples)+1) = mean(DataOutput{i}.q(:,j:j+nsamples-1),2);
                    DataUsed{i}.qdot(:,ceil(j/nsamples)+1) = mean(DataOutput{i}.qdot(:,j:j+nsamples-1),2);
                    DataUsed{i}.qdotdot(:,ceil(j/nsamples)+1) = mean(DataOutput{i}.qdotdot(:,j:j+nsamples-1),2);
                    DataUsed{i}.x.pos(:,ceil(j/nsamples)+1) = mean(DataOutput{i}.x.pos(:,j:j+nsamples-1),2);
                    DataUsed{i}.x.ori(:,ceil(j/nsamples)+1) = mean(DataOutput{i}.x.ori(:,j:j+nsamples-1),2);
                    DataUsed{i}.xdot.pos(:,ceil(j/nsamples)+1) = mean(DataOutput{i}.xdot.pos(:,j:j+nsamples-1),2);
                    DataUsed{i}.xdot.ori(:,ceil(j/nsamples)+1) = mean(DataOutput{i}.xdot.ori(:,j:j+nsamples-1),2);
                    DataUsed{i}.t(:,ceil(j/nsamples)+1) = mean(DataOutput{i}.t(:,j:j+nsamples-1),2);
                end

                % Add last pack of samples
                j=j+nsamples;
                DataUsed{i}.q(:,ceil(j/nsamples)+1) = mean(DataOutput{i}.q(:,j:end),2);
                DataUsed{i}.qdot(:,ceil(j/nsamples)+1) = mean(DataOutput{i}.qdot(:,j:end),2);
                DataUsed{i}.qdotdot(:,ceil(j/nsamples)+1) = mean(DataOutput{i}.qdotdot(:,j:end),2);
                DataUsed{i}.x.pos(:,ceil(j/nsamples)+1) = mean(DataOutput{i}.x.pos(:,j:end),2);
                DataUsed{i}.x.ori(:,ceil(j/nsamples)+1) = mean(DataOutput{i}.x.ori(:,j:end),2);
                DataUsed{i}.xdot.pos(:,ceil(j/nsamples)+1) = mean(DataOutput{i}.xdot.pos(:,j:end),2);
                DataUsed{i}.xdot.ori(:,ceil(j/nsamples)+1) = mean(DataOutput{i}.xdot.ori(:,j:end),2);
                DataUsed{i}.t(:,ceil(j/nsamples)+1) = mean(DataOutput{i}.t(:,j:end),2);

                % Add first sample
                DataUsed{i}.q(:,1) = DataOutput{i}.q(:,1);
                DataUsed{i}.qdot(:,1) = DataOutput{i}.qdot(:,1);
                DataUsed{i}.qdotdot(:,1) = DataOutput{i}.qdotdot(:,1);
                DataUsed{i}.x.pos(:,1) = DataOutput{i}.x.pos(:,1);
                DataUsed{i}.x.ori(:,1) = DataOutput{i}.x.ori(:,1);
                DataUsed{i}.xdot.pos(:,1) = DataOutput{i}.xdot.pos(:,1);
                DataUsed{i}.xdot.ori(:,1) = DataOutput{i}.xdot.ori(:,1);
                DataUsed{i}.t(:,1) = DataOutput{i}.t(:,1);

                % Add last sample
                DataUsed{i}.q(:,end) = DataOutput{i}.q(:,end);
                DataUsed{i}.qdot(:,end) = DataOutput{i}.qdot(:,end);
                DataUsed{i}.qdotdot(:,end) = DataOutput{i}.qdotdot(:,end);
                DataUsed{i}.x.pos(:,end) = DataOutput{i}.x.pos(:,end);
                DataUsed{i}.x.ori(:,end) = DataOutput{i}.x.ori(:,end);
                DataUsed{i}.xdot.pos(:,end) = DataOutput{i}.xdot.pos(:,end);
                DataUsed{i}.xdot.ori(:,end) = DataOutput{i}.xdot.ori(:,end);
                DataUsed{i}.t(:,end) = DataOutput{i}.t(:,end);
            end

            sizes=zeros(1,size(DataUsed,2));
            for i=1:size(DataUsed,2) %For each repetition
                sizes(i)=size(DataUsed{i}.t,2); %Check size of t
            end
            min_size = min(sizes);
            size_used = min_size - 1;
            tsample = mean(DataUsed{1}.t(2:end)-DataUsed{1}.t(1:end-1));
            obj.DataMean.t = linspace(0, size_used*tsample, size_used);
            obj.DataMin.t = obj.DataMean.t;
            obj.DataMax.t = obj.DataMean.t;

            obj.DataMean.q = zeros(7, size_used);
            obj.DataMean.qdot = zeros(7, size_used);
            obj.DataMean.qdotdot = zeros(7, size_used);
            obj.DataMin.q = zeros(7, size_used);
            obj.DataMin.qdot = zeros(7, size_used);
            obj.DataMin.qdotdot = zeros(7, size_used);
            obj.DataMax.q = zeros(7, size_used);
            obj.DataMax.qdot = zeros(7, size_used);
            obj.DataMax.qdotdot = zeros(7, size_used);

            q = zeros(size(DataUsed,2), size_used);
            qdot = zeros(size(DataUsed,2), size_used);
            qdotdot = zeros(size(DataUsed,2), size_used);
            for joint = 1:7
                for i = 1:size(DataUsed,2)
                    q(i,:) = DataUsed{i}.q(joint, 1:size_used);
                    qdot(i,:) = DataUsed{i}.qdot(joint, 1:size_used);
                    qdotdot(i,:) = DataUsed{i}.qdotdot(joint, 1:size_used);
                end
                obj.DataMean.q(joint,:) = mean(q,1);
                obj.DataMin.q(joint,:) = min(q,[],1);
                obj.DataMax.q(joint,:) = max(q,[],1);
                obj.DataMean.qdot(joint,:) = mean(qdot,1);
                obj.DataMin.qdot(joint,:) = min(qdot,[],1);
                obj.DataMax.qdot(joint,:) = max(qdot,[],1);
                obj.DataMean.qdotdot(joint,:) = mean(qdotdot,1);
                obj.DataMin.qdotdot(joint,:) = min(qdotdot,[],1);
                obj.DataMax.qdotdot(joint,:) = max(qdotdot,[],1);
            end
            obj.DataMean = fill_cartesian(obj.DataMean);
            obj.DataMin = fill_cartesian(obj.DataMin);
            obj.DataMax = fill_cartesian(obj.DataMax);
        end
        function CalculateMinMeanMaxJointError(obj, reference, DataUsed)
            % Calculate errors
            error.q = cell(1,7);
            error.qdot = cell(1,7);
            error.qdotdot = cell(1,7);
            error_tmp = cell(size(DataUsed,2),7);
            error_t = cell(size(DataUsed,2));
            
            for i=1:size(DataUsed,2) %For each repetition
                for njoint=1:7
                    [error_tmp{i, njoint}.q, error_t{i}] = get_error(reference.q(njoint,:),DataUsed{i}.q(njoint,:),reference.t,DataUsed{i}.t);
                    [error_tmp{i, njoint}.qdot, error_t{i}] = get_error(reference.qdot(njoint,:),DataUsed{i}.qdot(njoint,:),reference.t,DataUsed{i}.t);
                    [error_tmp{i, njoint}.qdotdot, error_t{i}] = get_error(reference.qdotdot(njoint,:),DataUsed{i}.qdotdot(njoint,:),reference.t,DataUsed{i}.t);
                end
            end
            sizes = zeros(1,size(DataUsed,2));
            for i = 1:size(DataUsed,2) %For each repetition
                sizes(i) = size(error_tmp{i,1}.q,2); %Check size of error
            end
            min_size = min(sizes);
            for i = 1:size(DataUsed,2) %For each repetition
                for njoint = 1:7
                    error.q{njoint}(i,:) = abs(error_tmp{i, njoint}.q(1:min_size));
                    error.qdot{njoint}(i,:) = abs(error_tmp{i, njoint}.qdot(1:min_size));
                    error.qdotdot{njoint}(i,:) = abs(error_tmp{i, njoint}.qdotdot(1:min_size));
                end
                error.t(i,:) = error_t{i}(1:min_size)';
            end
            
            obj.MeanError.t = mean(error.t,1);
            
            obj.MeanError.q = zeros(7, min_size);
            obj.MinError.q = zeros(7, min_size);
            obj.MaxError.q = zeros(7, min_size);
            obj.MeanError.q = zeros(7, min_size);
            obj.MinError.q = zeros(7, min_size);
            obj.MaxError.q = zeros(7, min_size);

            obj.MeanError.qdot = zeros(7, min_size);
            obj.MinError.qdot = zeros(7, min_size);
            obj.MaxError.qdot = zeros(7, min_size);
            obj.MeanError.qdot = zeros(7, min_size);
            obj.MinError.qdot = zeros(7, min_size);
            obj.MaxError.qdot = zeros(7, min_size);
            
            obj.MeanError.qdotdot = zeros(7, min_size);
            obj.MinError.qdotdot = zeros(7, min_size);
            obj.MaxError.qdotdot = zeros(7, min_size);
            obj.MeanError.qdotdot = zeros(7, min_size);
            obj.MinError.qdotdot = zeros(7, min_size);
            obj.MaxError.qdotdot = zeros(7, min_size);

            for i=1:7
                obj.MeanError.q(i,:) = mean(error.q{i},1);
                obj.MeanError.qdot(i,:) = mean(error.qdot{i},1);
                obj.MeanError.qdotdot(i,:) = mean(error.qdotdot{i},1);
                obj.MinError.q(i,:) = min(error.q{i},[],1);
                obj.MinError.qdot(i,:) = min(error.qdot{i},[],1);
                obj.MinError.qdotdot(i,:) = min(error.qdotdot{i},[],1);
                obj.MaxError.q(i,:) = max(error.q{i},[],1);
                obj.MaxError.qdot(i,:) = max(error.qdot{i},[],1);
                obj.MaxError.qdotdot(i,:) = max(error.qdotdot{i},[],1);
            end
        end
        function CalculateMinMeanMaxCartesianError(obj, reference, DataUsed)
            % Calculate errors
            error.x.pos=cell(1,3);
            error.xdot.pos=cell(1,3);
            error.x.ori=cell(1,3);
            error.xdot.ori=cell(1,3);
            error_tmp=cell(size(DataUsed,2),3);
            error_t = cell(size(DataUsed,2));

            for i=1:size(DataUsed,2) %For each repetition
                for ncoord=1:3
                    [error_tmp{i, ncoord}.x.pos, error_t{i}] = get_error(reference.x.pos(ncoord,:),DataUsed{i}.x.pos(ncoord,:),reference.t, DataUsed{i}.t);
                    [error_tmp{i, ncoord}.xdot.pos, error_t{i}] = get_error(reference.xdot.pos(ncoord,:),DataUsed{i}.xdot.pos(ncoord,:),reference.t, DataUsed{i}.t);
                    [error_tmp{i, ncoord}.x.ori, error_t{i}] = get_error(reference.x.ori(ncoord,:),DataUsed{i}.x.ori(ncoord,:),reference.t, DataUsed{i}.t);
                    [error_tmp{i, ncoord}.xdot.ori, error_t{i}] = get_error(reference.xdot.ori(ncoord,:),DataUsed{i}.xdot.ori(ncoord,:),reference.t, DataUsed{i}.t);
                end
            end
            
            sizes=zeros(1,size(DataUsed,2));
            for i=1:size(DataUsed,2) %For each repetition
                sizes(i)=size(error_tmp{i,1}.x.pos,2); %Check size of error
            end
            min_size = min(sizes);
            
            for i=1:size(DataUsed,2) %For each repetition
                for ncoord=1:3
                    error.x.pos{ncoord}(i,:)=abs(error_tmp{i,ncoord}.x.pos(1:min_size));
                    error.xdot.pos{ncoord}(i,:)=abs(error_tmp{i,ncoord}.xdot.pos(1:min_size));
                    error.x.ori{ncoord}(i,:)=abs(error_tmp{i,ncoord}.x.ori(1:min_size));
                    error.xdot.ori{ncoord}(i,:)=abs(error_tmp{i,ncoord}.xdot.ori(1:min_size));
                end
                error.t(i,:) = error_t{i}(1:min_size)';
            end
            
            obj.MeanError.t = mean(error.t,1);
            
            obj.MeanEucError.x.pos=zeros(1, min_size);
            obj.MinEucError.x.pos=zeros(1, min_size);
            obj.MaxEucError.x.pos=zeros(1, min_size);
            obj.MeanEucError.x.ori=zeros(1, min_size);
            obj.MinEucError.x.ori=zeros(1, min_size);
            obj.MaxEucError.x.ori=zeros(1, min_size);
            obj.MeanEucError.xdot.pos=zeros(1, min_size);
            obj.MinEucError.xdot.pos=zeros(1, min_size);
            obj.MaxEucError.xdot.pos=zeros(1, min_size);
            obj.MeanEucError.xdot.ori=zeros(1, min_size);
            obj.MinEucError.xdot.ori=zeros(1, min_size);
            obj.MaxEucError.xdot.ori=zeros(1, min_size);

            obj.MeanError.x.pos=zeros(3, min_size);
            obj.MinError.x.pos=zeros(3, min_size);
            obj.MaxError.x.pos=zeros(3, min_size);
            obj.MeanError.x.ori=zeros(3, min_size);
            obj.MinError.x.ori=zeros(3, min_size);
            obj.MaxError.x.ori=zeros(3, min_size);

            obj.MeanError.xdot.pos=zeros(3, min_size);
            obj.MinError.xdot.pos=zeros(3, min_size);
            obj.MaxError.xdot.pos=zeros(3, min_size);      
            obj.MeanError.xdot.ori=zeros(3, min_size);
            obj.MinError.xdot.ori=zeros(3, min_size);
            obj.MaxError.xdot.ori=zeros(3, min_size);
                        
            for i=1:3
                obj.MeanError.x.pos(i,:)=mean(error.x.pos{i},1);
                obj.MeanError.x.ori(i,:)=mean(error.x.ori{i},1);
                obj.MinError.x.pos(i,:)=min(error.x.pos{i},[],1);
                obj.MinError.x.ori(i,:)=min(error.x.ori{i},[],1);
                obj.MaxError.x.pos(i,:)=max(error.x.pos{i},[],1);
                obj.MaxError.x.ori(i,:)=max(error.x.ori{i},[],1);
                obj.MeanError.xdot.pos(i,:)=mean(error.xdot.pos{i},1);
                obj.MeanError.xdot.ori(i,:)=mean(error.xdot.ori{i},1);
                obj.MinError.xdot.pos(i,:)=min(error.xdot.pos{i},[],1);
                obj.MinError.xdot.ori(i,:)=min(error.xdot.ori{i},[],1);
                obj.MaxError.xdot.pos(i,:)=max(error.xdot.pos{i},[],1);
                obj.MaxError.xdot.ori(i,:)=max(error.xdot.ori{i},[],1);
            end
            obj.MeanEucError.x.pos=vecnorm(obj.MeanError.x.pos);
            obj.MaxEucError.x.pos=vecnorm(obj.MaxError.x.pos);
            obj.MinEucError.x.pos=vecnorm(obj.MinError.x.pos);
            obj.MeanEucError.x.ori=vecnorm(obj.MeanError.x.ori);
            obj.MaxEucError.x.ori=vecnorm(obj.MaxError.x.ori);
            obj.MinEucError.x.ori=vecnorm(obj.MinError.x.ori);
            obj.MeanEucError.xdot.pos=vecnorm(obj.MeanError.xdot.pos);
            obj.MaxEucError.xdot.pos=vecnorm(obj.MaxError.xdot.pos);
            obj.MinEucError.xdot.pos=vecnorm(obj.MinError.xdot.pos);
            obj.MeanEucError.xdot.ori=vecnorm(obj.MeanError.xdot.ori);
            obj.MaxEucError.xdot.ori=vecnorm(obj.MaxError.xdot.ori);
            obj.MinEucError.xdot.ori=vecnorm(obj.MinError.xdot.ori);
        end
        % Plot trajectories
        function ax = PlotJointPositionTrajectory(obj, Data, Color, bLimits, display)
            if (~isempty(Data))
                s = [1,3,5,7,2,4,6]; %subplot order
                ax = zeros(1,7);
                for i=1:7
                    ax(i) = subplot(4,2,s(i), 'Parent', display);
                    hold(ax(i), 'on');
                    plot(ax(i), Data.t, rad2deg(Data.q(i,:)), Color);
                    title(ax(i), ['Posici�n J', num2str(i)]);
                    ylabel(ax(i), 'q[�]');
                    xlabel(ax(i), 't[s]');
                    if (bLimits)
                        plot(ax(i), Data.t, ones(1, size(Data.t,2)).*rad2deg(obj.ThetaMax(i)), [Color, '--']);
                        plot(ax(i), Data.t, ones(1, size(Data.t,2)).*rad2deg(-obj.ThetaMax(i)), [Color, '--']);
                    end
                end
            end
        end
        function ax = PlotJointVelocityTrajectory(obj, Data, Color, bLimits, display)
            if (~isempty(Data))
                s = [1,3,5,7,2,4,6]; %subplot order
                ax = zeros(1, 7);
                for i=1:7
                    ax(i) = subplot(4,2,s(i), 'Parent', display);
                    hold(ax(i), 'on');
                    plot(ax(i), Data.t, rad2deg(Data.qdot(i,:)), Color);
                    title(ax(i), ['Velocidad J', num2str(i)]);
                    ylabel(ax(i), 'dq[�/s]');
                    xlabel(ax(i), 't[s]');
                    if (bLimits)
                        plot(ax(i), Data.t, ones(1, size(Data.t,2)).*rad2deg(obj.ThetaDotMax(i)), [Color, '--']);
                        plot(ax(i), Data.t, ones(1, size(Data.t,2)).*rad2deg(-obj.ThetaDotMax(i)), [Color, '--']);
                    end
                end
            end
        end
        function ax = PlotJointAccelerationTrajectory(obj, Data, Color, bLimits, display)
            if (~isempty(Data))
                s = [1,3,5,7,2,4,6]; %subplot order
                ax = zeros(1,7);
                for i=1:7
                    ax(i) = subplot(4,2,s(i), 'Parent', display);
                    hold(ax(i), 'on');
                    plot(ax(i), Data.t, rad2deg(Data.qdotdot(i,:)), Color);
                    title(ax(i), ['Aceleraci�n J', num2str(i)]);
                    ylabel(ax(i), 'ddq[�/s^2]');
                    xlabel(ax(i), 't[s]');
                    if (bLimits)
                        plot(ax(i), Data.t, ones(1, size(Data.t,2)).*rad2deg(obj.ThetaDotDotMax(i)), [Color, '--']);
                        plot(ax(i), Data.t, ones(1, size(Data.t,2)).*rad2deg(-obj.ThetaDotDotMax(i)), [Color, '--']);
                    end
                end
            end
        end
        function ax = PlotCartesianPositionTrajectory(obj, Data, Color, display)
            if (~isempty(Data))
                ax = zeros(1,6);
                for i=1:3
                    ax(i) = subplot(2,3,i, 'Parent', display);
                    hold(ax(i), 'on');
                    plot(ax(i), Data.t, 100*Data.x.pos(i,:), Color);
                    title(ax(i), ['Posici�n cartesiana ', obj.NameCoord(i)]);
                    ylabel(ax(i), 'x_{pos}[cm]');
                    xlabel(ax(i), 't[s]');
                end
                for i=1:3
                    ax(i+3) = subplot(2,3,i+3, 'Parent', display);
                    hold(ax(i+3), 'on');
                    plot(ax(i+3), Data.t, rad2deg(Data.x.ori(i,:)), Color);
                    title(ax(i+3), ['Cartesiana R', obj.NameCoord(3+i)]);
                    ylabel(ax(i+3), 'x_{ori}[�]');
                    xlabel(ax(i+3), 't[s]');
                end
            end
        end
        function Plot3DCartesianPositionTrajectory(obj, Data, Color, ax)
            if (~isempty(Data))
                plot3(ax, 100*Data.x.pos(1,:),100*Data.x.pos(2,:),100*Data.x.pos(3,:),'-', 'LineWidth', 1, 'Color',Color);
                axis(ax, 'equal');
                axis(ax, 'auto');
            end
        end
        function ax = PlotCartesianVelocityTrajectory(obj, Data, Color, display)
            if (~isempty(Data))
                ax = zeros(1,6);
                % Plot position cartesian velocity
                for i=1:3
                    ax(i) = subplot(2,3,i, 'Parent', display);
                    hold(ax(i), 'on');
                    plot(ax(i), Data.t, 100*Data.xdot.pos(i,:), Color);
                    title(ax(i), ['Velocidad ', obj.NameCoord(i)]);
                    ylabel(ax(i), 'dx_{pos}[cm/s]');
                    xlabel(ax(i), 't[s]');
                end
                % Plot orientation cartesian velocity
                for i=1:3
                    ax(i+3) = subplot(2,3,i+3, 'Parent', display);
                    hold(ax(i+3), 'on');
                    plot(ax(i+3), Data.t, rad2deg(Data.xdot.ori(i,:)), Color);
                    title(ax(i+3), ['Velocidad R', obj.NameCoord(3+i)]);
                    ylabel(ax(i+3), 'dx_{ori}[�/s]');
                    xlabel(ax(i+3), 't[s]');
                end
            end
        end
        % Plot trajectories repeatibility
        function ax = PlotJointPositionTrajectoryRepeatibility(obj, DataCommanded, display)
            x=[obj.DataMean.t, fliplr(obj.DataMean.t)];
            s = [1,3,5,7,2,4,6]; %subplot order
            ax = zeros(1,7);
            for j=1:7
                ax(j) = subplot(4,2,s(j), 'Parent', display);
                hold(ax(j), 'on');
                y=[rad2deg(obj.DataMax.q(j,:)), fliplr(rad2deg(obj.DataMin.q(j,:)))];
                fill(ax(j), x,y,obj.ColorLightRed);
                plot(ax(j), obj.DataMean.t, rad2deg(obj.DataMax.q(j,:)), obj.ColorDataOutput);
                plot(ax(j), obj.DataMean.t, rad2deg(obj.DataMin.q(j,:)), obj.ColorDataOutput);
                plot(ax(j), obj.DataMean.t, rad2deg(obj.DataMean.q(j,:)), obj.ColorDataOutput, 'LineWidth', 1);
                plot(ax(j), DataCommanded.t, rad2deg(DataCommanded.q(j,:)), [obj.ColorDataCommanded, '--'], 'LineWidth', 0.8);
                title(ax(j), strcat('Posici�n J', num2str(j)))
                ylabel(ax(j), 'q[�]');
                xlabel(ax(j), 't[s]');
            end
        end
        function ax = PlotJointVelocityTrajectoryRepeatibility(obj, DataCommanded, display)
            x=[obj.DataMean.t, fliplr(obj.DataMean.t)];
            s = [1,3,5,7,2,4,6]; %subplot order
            ax = zeros(1,7);
            for j=1:7
                ax(j) = subplot(4,2,s(j), 'Parent', display);
                hold(ax(j), 'on');
                y=[rad2deg(obj.DataMax.qdot(j,:)), fliplr(rad2deg(obj.DataMin.qdot(j,:)))];
                fill(ax(j), x,y,obj.ColorLightRed);
                plot(ax(j), obj.DataMax.t, rad2deg(obj.DataMax.qdot(j,:)), obj.ColorDataOutput);
                plot(ax(j), obj.DataMin.t, rad2deg(obj.DataMin.qdot(j,:)), obj.ColorDataOutput);
                plot(ax(j), obj.DataMean.t, rad2deg(obj.DataMean.qdot(j,:)), obj.ColorDataOutput, 'LineWidth', 1);
                plot(ax(j), DataCommanded.t, rad2deg(DataCommanded.qdot(j,:)), [obj.ColorDataCommanded, '--'], 'LineWidth', 0.8);
                title(ax(j), strcat('Velocidad J', num2str(j)))
                ylabel(ax(j), 'dq[�/s]');
                xlabel(ax(j), 't[s]');
            end
        end
        function ax = PlotJointAccelerationTrajectoryRepeatibility(obj, DataCommanded, display)
            x=[obj.DataMean.t, fliplr(obj.DataMean.t)];
            s = [1,3,5,7,2,4,6]; %subplot order
            ax = zeros(1,7);
            for j=1:7
                ax(j) = subplot(4,2,s(j), 'Parent', display);
                hold(ax(j), 'on');
                y=[rad2deg(obj.DataMax.qdotdot(j,:)), fliplr(rad2deg(obj.DataMin.qdotdot(j,:)))];
                fill(ax(j), x,y,obj.ColorLightRed);
                plot(ax(j), obj.DataMax.t, rad2deg(obj.DataMax.qdotdot(j,:)), obj.ColorDataOutput);
                plot(ax(j), obj.DataMin.t, rad2deg(obj.DataMin.qdotdot(j,:)), obj.ColorDataOutput);
                plot(ax(j), obj.DataMean.t, rad2deg(obj.DataMean.qdotdot(j,:)), obj.ColorDataOutput, 'LineWidth', 1);
                plot(ax(j), DataCommanded.t, rad2deg(DataCommanded.qdotdot(j,:)), [obj.ColorDataCommanded, '--'], 'LineWidth', 0.8);
                title(ax(j), strcat('Aceleraci�n J', num2str(j)))
                ylabel(ax(j), 'ddq[�/s^2]');
                xlabel(ax(j), 't[s]');
            end
        end
        function ax = PlotCartesianPositionTrajectoryRepeatibility(obj, DataCommanded, display)
            ax = zeros(1,6);
            x=[obj.DataMean.t, fliplr(obj.DataMean.t)];
            for xcoord=1:3
                ax(xcoord) = subplot(2,3,xcoord, 'Parent', display);
                hold(ax(xcoord), 'on');
                y=[100*(obj.DataMax.x.pos(xcoord,:)), fliplr(100*(obj.DataMin.x.pos(xcoord,:)))];
                fill(ax(xcoord), x,y,obj.ColorLightRed);
                plot(ax(xcoord), obj.DataMax.t, 100*(obj.DataMax.x.pos(xcoord,:)), obj.ColorDataOutput);
                plot(ax(xcoord), obj.DataMin.t, 100*(obj.DataMin.x.pos(xcoord,:)), obj.ColorDataOutput);
                plot(ax(xcoord), obj.DataMean.t, 100*(obj.DataMean.x.pos(xcoord,:)), obj.ColorDataOutput, 'LineWidth', 1);
                plot(ax(xcoord), DataCommanded.t, 100*(DataCommanded.x.pos(xcoord,:)), [obj.ColorDataCommanded, '--'], 'LineWidth', 0.8);
                title(ax(xcoord), ['Posici�n cartesiana ', obj.NameCoord(xcoord)]);
                ylabel(ax(xcoord), 'x_{pos}[cm]');
                xlabel(ax(xcoord), 't[s]');
            end
            for rxcoord=1:3
                ax(rxcoord+3) = subplot(2,3,rxcoord+3, 'Parent', display);
                hold(ax(rxcoord+3), 'on');
                y=[rad2deg(obj.DataMax.x.ori(rxcoord,:)), fliplr(rad2deg(obj.DataMin.x.ori(rxcoord,:)))];
                fill(ax(rxcoord+3), x,y,obj.ColorLightRed);
                plot(ax(rxcoord+3), obj.DataMax.t, rad2deg(obj.DataMax.x.ori(rxcoord,:)), obj.ColorDataOutput);
                plot(ax(rxcoord+3), obj.DataMin.t, rad2deg(obj.DataMin.x.ori(rxcoord,:)), obj.ColorDataOutput);
                plot(ax(rxcoord+3), obj.DataMean.t, rad2deg(obj.DataMean.x.ori(rxcoord,:)), obj.ColorDataOutput, 'LineWidth', 1);
                plot(ax(rxcoord+3), DataCommanded.t, rad2deg(DataCommanded.x.ori(rxcoord,:)), [obj.ColorDataCommanded, '--'], 'LineWidth', 0.8);
                title(ax(rxcoord+3), ['Cartesiana R', obj.NameCoord(rxcoord+3)]);
                ylabel(ax(rxcoord+3), 'x_{ori}[�]');
                xlabel(ax(rxcoord+3), 't[s]');
            end
        end
        function ax = PlotCartesianVelocityTrajectoryRepeatibility(obj, DataCommanded, display)
            ax = zeros(1,6);
            x=[obj.DataMean.t, fliplr(obj.DataMean.t)];
            for xcoord=1:3
                ax(xcoord) = subplot(2,3,xcoord, 'Parent', display);
                hold(ax(xcoord), 'on');
                y=[100*(obj.DataMax.xdot.pos(xcoord,:)), fliplr(100*(obj.DataMin.xdot.pos(xcoord,:)))];
                fill(ax(xcoord), x,y,obj.ColorLightRed);
                plot(ax(xcoord), obj.DataMax.t, 100*(obj.DataMax.xdot.pos(xcoord,:)), obj.ColorDataOutput);
                plot(ax(xcoord), obj.DataMin.t, 100*(obj.DataMin.xdot.pos(xcoord,:)), obj.ColorDataOutput);
                plot(ax(xcoord), obj.DataMean.t, 100*(obj.DataMean.xdot.pos(xcoord,:)), obj.ColorDataOutput, 'LineWidth', 1);
                plot(ax(xcoord), DataCommanded.t, 100*(DataCommanded.xdot.pos(xcoord,:)), [obj.ColorDataCommanded, '--'], 'LineWidth', 0.8);
                title(ax(xcoord), ['Velocidad ', obj.NameCoord(xcoord)]);
                ylabel(ax(xcoord), 'dx_{pos}[cm/s]');
                xlabel(ax(xcoord), 't[s]');
            end
            for rxcoord=1:3
                ax(rxcoord+3) = subplot(2,3,rxcoord+3, 'Parent', display);
                hold(ax(rxcoord+3), 'on');
                y=[rad2deg(obj.DataMax.xdot.ori(rxcoord,:)), fliplr(rad2deg(obj.DataMin.xdot.ori(rxcoord,:)))];
                fill(ax(rxcoord+3), x,y,obj.ColorLightRed);
                plot(ax(rxcoord+3), obj.DataMax.t, rad2deg(obj.DataMax.xdot.ori(rxcoord,:)), obj.ColorDataOutput);
                plot(ax(rxcoord+3), obj.DataMin.t, rad2deg(obj.DataMin.xdot.ori(rxcoord,:)), obj.ColorDataOutput);
                plot(ax(rxcoord+3), obj.DataMean.t, rad2deg(obj.DataMean.xdot.ori(rxcoord,:)), obj.ColorDataOutput, 'LineWidth', 1);
                plot(ax(rxcoord+3), DataCommanded.t, rad2deg(DataCommanded.xdot.ori(rxcoord,:)), [obj.ColorDataCommanded, '--'], 'LineWidth', 0.8);
                title(ax(rxcoord+3), ['Velocidad R', obj.NameCoord(rxcoord+3)]);
                ylabel(ax(rxcoord+3), 'dx_{ori}[�/s]');
                xlabel(ax(rxcoord+3), 't[s]');
            end
        end
        % Plot execution error
        function PlotJointPositionError_(obj, Data1, Data2, display)
            if (~isempty(Data1) && ~isempty(Data2))
                sgtitle(display, 'Error posici�n articular');
                if(strcmp(display.Type, 'uitab'))
                    ax = subplot(1,1,1, 'Parent', display);%uiaxes(display, 'Position', display.InnerPosition*0.92);
                else
                    display.Name = 'Error posici�n articular';
                    display.WindowState = 'maximized';
                    ax = axes;
                end
                hold(ax, 'on');
                rms_q = zeros(1,7);
                for i=1:7
                    hold(ax, 'on');
                    [q_error, t_error]=get_error(Data1.q(i,:), Data2.q(i,:), Data1.t, Data2.t);
                    rms_q(i)=rms(q_error);
                    plot(ax, t_error, rad2deg(q_error), obj.ColorsJoints(i));
                    dashedcolor=['--', obj.ColorsJoints(i)];
                    plot(ax, t_error, ones(size(t_error))*rad2deg(rms_q(i)), dashedcolor);
                end
                legend(ax, 'J1', ['rms_{J1} = ', num2str(rad2deg(rms_q(1))), '�'],'J2', ['rms_{J2} = ', num2str(rad2deg(rms_q(2))), '�'], 'J3', ['rms_{J3} = ', num2str(rad2deg(rms_q(3))), '�'], ...
                     'J4', ['rms_{J4} = ', num2str(rad2deg(rms_q(4))), '�'], 'J5', ['rms_{J5} = ', num2str(rad2deg(rms_q(5))), '�'], 'J6', ['rms_{J6} = ', num2str(rad2deg(rms_q(6))), '�'], ...
                     'J7', ['rms_{J7} = ', num2str(rad2deg(rms_q(7))), '�']);
                title(ax, ['Error medio posici�n articular = ', num2str(rad2deg(mean(rms_q))), '�']);
                ylabel(ax, 'E_{p}(q)[�]');
                xlabel(ax, 't[s]');
            end
        end
        function PlotJointVelocityError_(obj, Data1, Data2, display)
        	if (~isempty(Data1) && ~isempty(Data2))
                sgtitle(display, 'Error velocidad articular');
                if(strcmp(display.Type, 'uitab'))
                    ax = subplot(1,1,1,'Parent', display);%uiaxes(display, 'Position', display.InnerPosition*0.92);
                else
                    display.Name = 'Error velocidad articular';
                    display.WindowState = 'maximized';
                    ax = axes;
                end
                hold(ax, 'on');
                rms_qdot = zeros(1,7);
                for i=1:7
                    hold(ax, 'on');
                    [qdot_error, t_error]=get_error(Data1.qdot(i,:), Data2.qdot(i,:), Data1.t, Data2.t);
                    rms_qdot(i)=rms(qdot_error);
                    plot(ax, t_error, rad2deg(qdot_error), obj.ColorsJoints(i));
                    dashedcolor=['--', obj.ColorsJoints(i)];
                    plot(ax, t_error, ones(size(t_error))*rad2deg(rms_qdot(i)), dashedcolor);
                end
                legend(ax, 'J1', ['rms_{J1} = ', num2str(rad2deg(rms_qdot(1))), '�/s'],'J2', ['rms_{J2} = ', num2str(rad2deg(rms_qdot(2))), '�/s'], 'J3', ['rms_{J3} = ', num2str(rad2deg(rms_qdot(3))), '�/s'], ...
                     'J4', ['rms_{J4} = ', num2str(rad2deg(rms_qdot(4))), '�/s'], 'J5', ['rms_{J5} = ', num2str(rad2deg(rms_qdot(5))), '�/s'], 'J6', ['rms_{J6} = ', num2str(rad2deg(rms_qdot(6))), '�/s'], ...
                     'J7', ['rms_{J7} = ', num2str(rad2deg(rms_qdot(7))), '�/s']);
                title(ax, ['Error medio velocidad articular = ', num2str(rad2deg(mean(rms_qdot))), '�/s']);
                ylabel(ax, 'E_{p}(dq)[�/s]');
                xlabel(ax, 't[s]');
        	end
        end
        function PlotJointAccelerationError_(obj, Data1, Data2, display)
        	if (~isempty(Data1) && ~isempty(Data2))
                sgtitle(display, 'Error aceleraci�n articular');
                if(strcmp(display.Type, 'uitab'))
                    ax = subplot(1,1,1,'Parent', display);%uiaxes(display, 'Position', display.InnerPosition*0.92);
                else
                    display.Name = 'Error aceleraci�n articular';
                    display.WindowState = 'maximized';
                    ax = axes;
                end
                hold(ax, 'on');
                rms_qdotdot = zeros(1,7);
                for i=1:7
                    hold(ax, 'on');
                    [qdotdot_error, t_error]=get_error(Data1.qdotdot(i,:), Data2.qdotdot(i,:), Data1.t, Data2.t);
                    rms_qdotdot(i)=rms(qdotdot_error);
                    plot(ax, t_error, rad2deg(qdotdot_error), obj.ColorsJoints(i));
                    dashedcolor=['--', obj.ColorsJoints(i)];
                    plot(ax, t_error, ones(size(t_error))*rad2deg(rms_qdotdot(i)), dashedcolor);
                end
                legend(ax, 'J1', ['rms_{J1} = ', num2str(rad2deg(rms_qdotdot(1))), '�/s^2'],'J2', ['rms_{J2} = ', num2str(rad2deg(rms_qdotdot(2))), '�/s^2'], 'J3', ['rms_{J3} = ', num2str(rad2deg(rms_qdotdot(3))), '�/s^2'], ...
                     'J4', ['rms_{J4} = ', num2str(rad2deg(rms_qdotdot(4))), '�/s^2'], 'J5', ['rms_{J5} = ', num2str(rad2deg(rms_qdotdot(5))), '�/s^2'], 'J6', ['rms_{J6} = ', num2str(rad2deg(rms_qdotdot(6))), '�/s^2'], ...
                     'J7', ['rms_{J7} = ', num2str(rad2deg(rms_qdotdot(7))), '�/s^2']);
                title(ax, ['Error medio aceleraci�n articular = ', num2str(rad2deg(mean(rms_qdotdot))), '�/s^2']);
                ylabel(ax, 'E_{p}(ddq)[�/s^2]');
                xlabel(ax, 't[s]');
        	end
        end
        function PlotCartesianPositionError_(obj, Data1, Data2, display)
            if (~isempty(Data1) && ~isempty(Data2))
                sgtitle(display, 'Error posici�n cartesiana');
                ax(1) = subplot(1,2,1, 'Parent', display);
                hold(ax(1), 'on');
                % Get position cartesian error
                [xpos_error, t_error]=get_error(Data1.x.pos, Data2.x.pos, Data1.t, Data2.t);
                rms_xpos = zeros(1,3);
                for i=1:3
                    rms_xpos(i)=rms(xpos_error(i,:));
                    plot(ax(1), t_error, 100*(xpos_error(i,:)), obj.ColorsXYZ(i));
                    dashedcolor=['--', obj.ColorsXYZ(i)];
                    plot(ax(1), t_error, ones(size(t_error))*rad2deg(rms_xpos(i)), dashedcolor);
                end
                xpos_euc_error=vecnorm(xpos_error);
                rms_euc_xpos=rms(xpos_euc_error);
                plot(ax(1), t_error, 100*(xpos_euc_error), 'k');
                dashedcolor=['--', 'k'];
                plot(ax(1), t_error, ones(size(t_error))*rad2deg(rms_euc_xpos), dashedcolor);
                % Show rms values in legend
                legend(ax(1), 'X', ['rms_{X} = ', num2str(100*rms_xpos(1),3), 'cm'], 'Y', ['rms_{Y} = ', num2str(100*rms_xpos(2),3), 'cm'], 'Z', ['rms_{Z} = ', num2str(100*rms_xpos(3),3), 'cm'], 'Euclidean', ['rms_{euc}=', num2str(100*rms_euc_xpos,3), 'cm']);
                % And euclidean rms value in the title
                title(ax(1), ['Error medio posici�n cartesiana = ', num2str(100*rms_euc_xpos,3), 'cm']);
                ylabel(ax(1), 'E_{p}(x_{pos})[cm]');
                xlabel(ax(1), 't[s]');
                
                ax(2) = subplot(1,2,2, 'Parent', display);
                hold(ax(2), 'on');
                % Get orientation cartesian error
                [xori_error, t_error]=get_error_ori(Data1.x.ori, Data2.x.ori, Data1.t, Data2.t);
                rms_xori = zeros(1,3);
                for i=1:3
                    rms_xori(i)=rms(xori_error(i,:));
                    plot(ax(2), t_error, 100*(xori_error(i,:)), obj.ColorsXYZ(i));
                    dashedcolor=['--', obj.ColorsXYZ(i)];
                    plot(ax(2), t_error, ones(size(t_error))*rad2deg(rms_xori(i)), dashedcolor);
                end
                xori_euc_error=vecnorm(xori_error);
                rms_euc_xori=rms(xori_euc_error);
                plot(ax(2), t_error, 100*(xori_euc_error), 'k');
                dashedcolor=['--', 'k'];
                plot(ax(2), t_error, ones(size(t_error))*rad2deg(rms_euc_xori), dashedcolor);
                % Show rms values in the legend
                legend(ax(2), 'RX', ['rms_{RX} = ', num2str(rad2deg(rms_xori(1)),3), '�'], 'RY', ['rms_{RY} = ', num2str(rad2deg(rms_xori(2)),3), '�'], 'RZ', ['rms_{RZ} = ', num2str(rad2deg(rms_xori(3)),3), '�'], 'Euclidean', ['rms_{euc}=', num2str(rad2deg(rms_euc_xpos),3), '�']);
                title(ax(2), ['Error medio orientaci�n cartesiana = ', num2str(rad2deg(rms_euc_xori),3), '�']);
                % And the rms euclidean value in the title
                ylabel(ax(2), 'E_{p}(x_{ori})[cm]');
                xlabel(ax(2), 't[s]');
            end
        end
        function PlotCartesianVelocityError_(obj, Data1, Data2, display)
            if (~isempty(Data1) && ~isempty(Data2))
                sgtitle(display, 'Error velocidad cartesiana');
                ax(1) = subplot(1,2,1, 'Parent', display);
                hold(ax(1), 'on');
                % Plot position cartesian velocity error
                [xdotpos_error, t_error]=get_error(Data1.xdot.pos, Data2.xdot.pos, Data1.t, Data2.t);
                rms_xdotpos = zeros(1,3);
                for i=1:3
                    rms_xdotpos(i)=rms(xdotpos_error(i,:));
                    plot(ax(1), t_error, 100*(xdotpos_error(i,:)), obj.ColorsXYZ(i));
                    dashedcolor=['--', obj.ColorsXYZ(i)];
                    plot(ax(1), t_error, ones(size(t_error))*rad2deg(rms_xdotpos(i)), dashedcolor);
                end
                xdotpos_euc_error=vecnorm(xdotpos_error);
                rms_euc_xdotpos=rms(xdotpos_euc_error);
                plot(ax(1), t_error, 100*(xdotpos_euc_error), 'k');
                dashedcolor=['--', 'k'];
                plot(ax(1), t_error, ones(size(t_error))*rad2deg(rms_euc_xdotpos), dashedcolor);
                % Show the rms values in the legend
                legend(ax(1), 'X', ['rms_{X} = ', num2str(100*rms_xdotpos(1)), 'cm/s'], 'Y', ['rms_{Y} = ', num2str(100*rms_xdotpos(2)), 'cm/s'], 'Z', ['rms_{Z} = ', num2str(100*rms_xdotpos(3)), 'cm/s'], 'Euclidean', ['rms_{euc}=', num2str(100*rms_euc_xdotpos), 'cm/s']);
                % And the rms euclidean cartesian error in the title
                title(ax(1), ['Error medio posici�n vel. cartesiana = ', num2str(100*rms_euc_xdotpos), 'cm/s']);
                % Labels
                ylabel(ax(1), 'E_{p}(dx_{pos})[cm]');
                xlabel(ax(1), 't[s]');
                
                ax(2) = subplot(1,2,2, 'Parent', display);
                hold(ax(2), 'on');
                % Plot orientation cartesian velocity error
                [xdotori_error, t_error]=get_error(Data1.xdot.ori, Data2.xdot.ori, Data1.t, Data2.t);
                rms_xdotori = zeros(1,3);
                for i=1:3
                    rms_xdotori(i)=rms(xdotori_error(i,:));
                    plot(ax(2), t_error, 100*(xdotori_error(i,:)), obj.ColorsXYZ(i));
                    dashedcolor=['--', obj.ColorsXYZ(i)];
                    plot(ax(2), t_error, ones(size(t_error))*rad2deg(rms_xdotori(i)), dashedcolor);
                end
                xdotori_euc_error=vecnorm(xdotori_error);
                rms_euc_xdotori=rms(xdotori_euc_error);
                plot(ax(2), t_error, 100*(xdotori_euc_error), 'k');
                dashedcolor=['--', 'k'];
                plot(ax(2), t_error, ones(size(t_error))*rad2deg(rms_euc_xdotori), dashedcolor);
                % Show the rms values in the legend
                legend(ax(2), 'RX', ['rms_{RX} = ', num2str(rad2deg(rms_xdotori(1))), '�/s'], 'RY', ['rms_{RY} = ', num2str(rad2deg(rms_xdotori(2))), '�/s'], 'RZ', ['rms_{RZ} = ', num2str(rad2deg(rms_xdotori(3))), '�/s'], 'Euclidean', ['rms_{euc}=', num2str(rad2deg(rms_euc_xdotpos)), 'º/s']);
                % And the rms euclidean orientation error in the title
                title(ax(2), ['Error medio orientaci�n vel. cartesiana = ', num2str(rad2deg(rms_euc_xdotori)), '�/s']);  
                % Labels
                ylabel(ax(2), 'E_{p}(dx_{ori})[cm]');
                xlabel(ax(2), 't[s]');
            end
        end
        % Plot repeatibility
        function PlotJointPositionRepeatibility(obj, display)
            x=[obj.MeanError.t, fliplr(obj.MeanError.t)];
            s = [1,3,5,7,2,4,6]; %subplot order
            ax = zeros(1,7);
            for j=1:7
                ax(j) = subplot(4,2,s(j), 'Parent', display);
                hold(ax(j), 'on');
                y=[rad2deg(obj.MaxError.q(j,:)), fliplr(rad2deg(obj.MinError.q(j,:)))];
                fill(ax(j), x,y,obj.ColorLightRed);
                plot(ax(j), obj.MeanError.t, rad2deg(obj.MaxError.q(j,:)), obj.ColorDataOutput);
                plot(ax(j), obj.MeanError.t, rad2deg(obj.MinError.q(j,:)), obj.ColorDataOutput);
                plot(ax(j), obj.MeanError.t, rad2deg(obj.MeanError.q(j,:)), obj.ColorDataOutput, 'LineWidth', 2);
                title(ax(j), strcat('Error posici�n J', num2str(j), ' = ', num2str(rms(rad2deg(obj.MeanError.q(j,:)))),'�'))
                ylabel(ax(j), 'E(q)[�]');
                xlabel(ax(j), 't[s]');
            end
            f = get(ax(end), 'Children');
            legend(f(1), 'Error medio')
        end
        function PlotJointVelocityRepeatibility(obj, display)
        	x=[obj.MeanError.t, fliplr(obj.MeanError.t)];
            s = [1,3,5,7,2,4,6]; %subplot order
            ax = zeros(1,7);
            for j=1:7
                ax(j) = subplot(4,2,s(j), 'Parent', display);
                hold(ax(j), 'on');
                y=[rad2deg(obj.MaxError.qdot(j,:)), fliplr(rad2deg(obj.MinError.qdot(j,:)))];
                fill(ax(j), x,y,obj.ColorLightRed);
                plot(ax(j), obj.MeanError.t, rad2deg(obj.MaxError.qdot(j,:)), obj.ColorDataOutput);
                plot(ax(j), obj.MeanError.t, rad2deg(obj.MinError.qdot(j,:)), obj.ColorDataOutput);
                plot(ax(j), obj.MeanError.t, rad2deg(obj.MeanError.qdot(j,:)), obj.ColorDataOutput, 'LineWidth', 2);
                title(ax(j), strcat('Error velocidad J', num2str(j), ' = ', num2str(rms(rad2deg(obj.MeanError.qdot(j,:)))),'�/s'))
                ylabel(ax(j), 'E(dq)[�/s]');
                xlabel(ax(j), 't[s]');
            end
            f = get(ax(end), 'Children');
            legend(f(1), 'Error medio');
        end
        function PlotJointAccelerationRepeatibility(obj, display)
        	x=[obj.MeanError.t, fliplr(obj.MeanError.t)];
            s = [1,3,5,7,2,4,6]; %subplot order
            ax = zeros(1,7);
            for j=1:7
                ax(j) = subplot(4,2,s(j), 'Parent', display);
                hold(ax(j), 'on');
                y=[rad2deg(obj.MaxError.qdotdot(j,:)), fliplr(rad2deg(obj.MinError.qdotdot(j,:)))];
                fill(ax(j), x,y,obj.ColorLightRed);
                plot(ax(j), obj.MeanError.t, rad2deg(obj.MaxError.qdotdot(j,:)), obj.ColorDataOutput);
                plot(ax(j), obj.MeanError.t, rad2deg(obj.MinError.qdotdot(j,:)), obj.ColorDataOutput);
                plot(ax(j), obj.MeanError.t, rad2deg(obj.MeanError.qdotdot(j,:)), obj.ColorDataOutput, 'LineWidth', 2);
                title(ax(j), strcat('Error aceleraci�n J', num2str(j), ' = ', num2str(rms(rad2deg(obj.MeanError.qdotdot(j,:)))),'�/s^2'))
                ylabel(ax(j), 'E(ddq)[�/s^2]');
                xlabel(ax(j), 't[s]');
            end
            f = get(ax(end), 'Children');
            legend(f(1), 'Error medio')
        end
        function PlotCartesianPositionRepeatibility(obj, display)
%             ax = zeros(1,8);
            x=[obj.MeanError.t, fliplr(obj.MeanError.t)];
            for xcoord=1:3
                ax(xcoord) = subplot(6,2,2*xcoord-1, 'Parent', display);
                ax(xcoord).ActivePositionProperty = 'outerposition';
                hold(ax(xcoord), 'on');
                y=[rad2deg(obj.MaxError.x.pos(xcoord,:)), fliplr(rad2deg(obj.MinError.x.pos(xcoord,:)))];
                fill(ax(xcoord), x,y,obj.ColorLightBlue);
                plot(ax(xcoord), obj.MeanError.t, rad2deg(obj.MaxError.x.pos(xcoord,:)), 'b');
                plot(ax(xcoord), obj.MeanError.t, rad2deg(obj.MinError.x.pos(xcoord,:)), 'b');
                plot(ax(xcoord), obj.MeanError.t, rad2deg(obj.MeanError.x.pos(xcoord,:)), 'b', 'LineWidth', 2);
                title(ax(xcoord), ['Error cartesiano  ', obj.NameCoord(xcoord),' = ', num2str(rms(100*obj.MeanError.x.pos(xcoord,:))), 'cm'], 'FontSize', 10.5)
                ylabel(ax(xcoord), 'E(x)[cm]')
                xlabel(ax(xcoord), 't[s]')
            end
            ax(4) = subplot(2,2,2, 'Parent', display);
            hold(ax(4), 'on');
            y=[100*obj.MaxEucError.x.pos, fliplr(100*obj.MinEucError.x.pos)];
            fill(ax(4), x,y,obj.ColorLightBlue);
            plot(ax(4), obj.MeanError.t, 100*obj.MaxEucError.x.pos, 'b');
            plot(ax(4), obj.MeanError.t, 100*obj.MinEucError.x.pos, 'b');
            plot(ax(4), obj.MeanError.t, 100*obj.MeanEucError.x.pos, 'b', 'LineWidth', 2);
            title(ax(4), ['Error posici�n cartesiana = ', num2str(rms(100*obj.MeanEucError.x.pos)), 'cm'])
            ylabel(ax(4), 'E(x)[cm]')
            xlabel(ax(4), 't[s]')
            f = get(ax(4), 'Children');
            legend(f(1), 'Error medio')
            
            %x.ori
            for rxcoord=1:3
                ax(rxcoord+4) = subplot(6,2,2*(rxcoord+3)-1, 'Parent', display);
                ax(rxcoord+4).ActivePositionProperty = 'outerposition';
                hold(ax(rxcoord+4), 'on');
                y=[rad2deg(obj.MaxError.x.ori(rxcoord,:)), fliplr(rad2deg(obj.MinError.x.ori(rxcoord,:)))];
                fill(ax(rxcoord+4), x,y,obj.ColorLightRed);
                plot(ax(rxcoord+4), obj.MeanError.t, rad2deg(obj.MaxError.x.ori(rxcoord,:)), 'r');
                plot(ax(rxcoord+4), obj.MeanError.t, rad2deg(obj.MinError.x.ori(rxcoord,:)), 'r');
                plot(ax(rxcoord+4), obj.MeanError.t, rad2deg(obj.MeanError.x.ori(rxcoord,:)), 'r', 'LineWidth', 2);
                title(ax(rxcoord+4), ['Error cartesiano R', obj.NameCoord(rxcoord+3),' = ', num2str(rms(rad2deg(obj.MeanError.x.ori(rxcoord,:)))), '�'], 'FontSize', 10.5)
                ylabel(ax(rxcoord+4), 'E(x)[�]')
                xlabel(ax(rxcoord+4), 't[s]')
            end
            ax(8) = subplot(2,2,4, 'Parent', display);
            hold(ax(8), 'on');
            y=[rad2deg(obj.MaxEucError.x.ori), fliplr(rad2deg(obj.MinEucError.x.ori))];
            fill(ax(8), x,y,obj.ColorLightRed);
            plot(ax(8), obj.MeanError.t, rad2deg(obj.MaxEucError.x.ori), 'r');
            plot(ax(8), obj.MeanError.t, rad2deg(obj.MinEucError.x.ori), 'r');
            plot(ax(8), obj.MeanError.t, rad2deg(obj.MeanEucError.x.ori), 'r', 'LineWidth', 2);
            title(ax(8), ['Error orientaci�n cartesiana = ', num2str(rms(rad2deg(obj.MeanEucError.x.ori))), '�'])
            ylabel(ax(8), 'E(x)[�]')
            xlabel(ax(8), 't[s]')
            f = get(ax(8), 'Children');
            legend(f(1), 'Error medio');
        end
        function PlotCartesianVelocityRepeatibility(obj, display)
%             ax = zeros(1,8);
           	x=[obj.MeanError.t, fliplr(obj.MeanError.t)];
            for xcoord=1:3
                ax(xcoord) = subplot(6,2,2*xcoord-1, 'Parent', display);
                ax(xcoord).ActivePositionProperty = 'outerposition';
                hold(ax(xcoord), 'on');
                y=[rad2deg(obj.MaxError.xdot.pos(xcoord,:)), fliplr(rad2deg(obj.MinError.xdot.pos(xcoord,:)))];
                fill(ax(xcoord), x,y,obj.ColorLightBlue);
                plot(ax(xcoord), obj.MeanError.t, rad2deg(obj.MaxError.xdot.pos(xcoord,:)), 'b');
                plot(ax(xcoord), obj.MeanError.t, rad2deg(obj.MinError.xdot.pos(xcoord,:)), 'b');
                plot(ax(xcoord), obj.MeanError.t, rad2deg(obj.MeanError.xdot.pos(xcoord,:)), 'b', 'LineWidth', 2);
                title(ax(xcoord), ['Error velocidad cartesiana ', obj.NameCoord(xcoord),' = ', num2str(rms(100*obj.MeanError.xdot.pos(xcoord,:))), 'cm/s'], 'FontSize', 10.5)
                ylabel(ax(xcoord), 'E(dx)[cm]')
                xlabel(ax(xcoord), 't[s]')
            end
            ax(4) = subplot(2,2,2, 'Parent', display);
            hold(ax(4), 'on');
            y=[100*obj.MaxEucError.xdot.pos, fliplr(100*obj.MinEucError.xdot.pos)];
            fill(ax(4), x,y,obj.ColorLightBlue);
            plot(ax(4), obj.MeanError.t, 100*obj.MaxEucError.xdot.pos, 'b');
            plot(ax(4), obj.MeanError.t, 100*obj.MinEucError.xdot.pos, 'b');
            plot(ax(4), obj.MeanError.t, 100*obj.MeanEucError.xdot.pos, 'b', 'LineWidth', 2);
            title(ax(4), ['Error posici�n vel. cartesiana = ', num2str(rms(100*obj.MeanEucError.xdot.pos)), 'cm/s'])
            ylabel(ax(4), 'E(dx)[cm]')
            xlabel(ax(4), 't[s]')
            f = get(ax(4), 'Children');
            legend(f(1), 'Error medio')
            
            %xdot.ori
            for rxcoord=1:3
                ax(rxcoord+4) = subplot(6,2,2*(rxcoord+3)-1, 'Parent', display);
                ax(rxcoord+4).ActivePositionProperty = 'outerposition';
                hold(ax(rxcoord+4), 'on');
                y=[rad2deg(obj.MaxError.xdot.ori(rxcoord,:)), fliplr(rad2deg(obj.MinError.xdot.ori(rxcoord,:)))];
                fill(ax(rxcoord+4), x,y,obj.ColorLightRed);
                plot(ax(rxcoord+4), obj.MeanError.t, rad2deg(obj.MaxError.xdot.ori(rxcoord,:)), 'r');
                plot(ax(rxcoord+4), obj.MeanError.t, rad2deg(obj.MinError.xdot.ori(rxcoord,:)), 'r');
                plot(ax(rxcoord+4), obj.MeanError.t, rad2deg(obj.MeanError.xdot.ori(rxcoord,:)), 'r', 'LineWidth', 2);
                title(ax(rxcoord+4), ['Error velocidad cartesiana R', obj.NameCoord(rxcoord+3),' = ', num2str(rms(rad2deg(obj.MeanError.xdot.ori(rxcoord,:)))), '�/s'], 'FontSize', 10.5)
                ylabel(ax(rxcoord+4), 'E(dx)[�]')
                xlabel(ax(rxcoord+4), 't[s]')
            end
            ax(8) = subplot(2,2,4, 'Parent', display);
            hold(ax(8), 'on');
            y=[rad2deg(obj.MaxEucError.xdot.ori), fliplr(rad2deg(obj.MinEucError.xdot.ori))];
            fill(ax(8), x,y,obj.ColorLightRed);
            plot(ax(8), obj.MeanError.t, rad2deg(obj.MaxEucError.xdot.ori), 'r');
            plot(ax(8), obj.MeanError.t, rad2deg(obj.MinEucError.xdot.ori), 'r');
            plot(ax(8), obj.MeanError.t, rad2deg(obj.MeanEucError.xdot.ori), 'r', 'LineWidth', 2);
            title(ax(8), ['Error orientaci�n vel. cartesiana = ', num2str(rms(rad2deg(obj.MeanEucError.xdot.ori))), '�/s'])
            ylabel(ax(8), 'E(dx)[�]')
            xlabel(ax(8), 't[s]')
            f = get(ax(8), 'Children');
            legend(f(1), 'Error medio')
        end
    end
    methods(Static, Access=public) 
        function [output, commanded] = SetTrajectoryFromFile(Trajectory)
            if (size(Trajectory,1)==1) && (size(Trajectory,2)==1)
                output.t = Trajectory.Timestamps;
                output.q = Trajectory.JointTrajectory;
                output.x.pos = Trajectory.CartesianTrajectory.Position;
                output.x.ori = Trajectory.CartesianTrajectory.Orientation;
                output.qdot = Trajectory.JointVelocities;
                output.xdot.pos = Trajectory.CartesianVelocities.Position;
                output.xdot.ori = Trajectory.CartesianVelocities.Orientation;
                output.qdotdot = Trajectory.JointAccelerations;
                output.t_torque = Trajectory.JointTorques.Timestamps;
                output.q_torque = Trajectory.JointTorques.Torques;
                commanded = Trajectory.DataCommanded;
            else
                output{max(size(Trajectory))} = [];
                commanded{max(size(Trajectory))} = [];
                for i = 1:max(size(Trajectory))
                    output{i}.t = Trajectory{i}.Timestamps;
                    output{i}.q = Trajectory{i}.JointTrajectory;
                    output{i}.x.pos = Trajectory{i}.CartesianTrajectory.Position;
                    output{i}.x.ori = Trajectory{i}.CartesianTrajectory.Orientation;
                    output{i}.qdot = Trajectory{i}.JointVelocities;
                    output{i}.xdot.pos = Trajectory{i}.CartesianVelocities.Position;
                    output{i}.xdot.ori = Trajectory{i}.CartesianVelocities.Orientation;
                    output{i}.qdotdot = Trajectory{i}.JointAccelerations;
                    output{i}.t_torque = Trajectory{i}.JointTorques.Timestamps;
                    output{i}.q_torque = Trajectory{i}.JointTorques.Torques;
                    commanded{i} = Trajectory{i}.DataCommanded;
                end
            end
        end
        % ADJUST TRAJECTORY FUNCTIONS
        function DataSmooth = SmoothData(Data, velocity, smoothFactor)
            NoPausesData=delete_pause_init_end(Data);
            %Smooth the data
            DataSmooth=bounded_spline(NoPausesData, velocity, smoothFactor);
        end
    end
end

