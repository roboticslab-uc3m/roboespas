classdef DelsysSensors < handle
    %DELSYSSENSORS 
    properties (Access = public)
        DelsysTrial
        DelsysReference
        
        Movement 
        SensorsPosition = zeros(16,3);
        
        EMG % EMG.Values && EMG.Timestamps 
        IMU % IMU.Values && IMU.Timestamps
        
        DelsysComm % DelsysCommunication object
                        
        % Repeatibility properties
        DataMin
        DataMean
        DataMax
    end
    properties (GetAccess = public, SetAccess = private)
        NumSensors % Number of sensors used in the specified movement
        SensorsMuscles
        DelsysSensorsConnected
    end
    properties (Access = public, Constant)
    	DelsysSensorsPropertiesMap = containers.Map({'flexext', 'prosup'}, ...
            {{4,{'Biceps Brachii', 'Triceps Lateral Head', 'Brachioradialis', 'Flexor Carpi Radialis'}}, {1,{'noMuscles'}}});
    end
    properties (Access = private, Constant)
        ColorLightBlue = [152/256, 163/256, 243/256];
        ColorLightRed = [243/256, 163/256, 152/256];
        ColorLightGreen = [163/256, 243/256, 152/256];
        
        gToMs2 = 9.79956; %9.80665; % Conversion from [g] to [m/s2]
    end
    
    methods (Access = public)
        % Object definition
        function obj = DelsysSensors(DelsysSensorsConnected)
            %DELSYSSENSORS Construct an instance of this class only if the
            %sensors are connected 
            obj.DelsysSensorsConnected = DelsysSensorsConnected;
            if (obj.DelsysSensorsConnected)
                % Create DelsysCommunication object
                obj.DelsysComm = DelsysCommunication(1);  
            end
        end      
        function SetMovement(obj, movement)
            properties = obj.DelsysSensorsPropertiesMap(movement);
            obj.NumSensors = properties{1,1};
            obj.SensorsMuscles = properties{1,2};
            obj.Movement = movement;
        end
        function SetSensorsPosition(obj, posArray)
            obj.SensorsPosition = posArray;
        end
        
        % Sensors control
        function StartSensors(obj)
            % Start sensors data streaming. Must wait 3 seconds after calling the function (it's non-blocking)
            if(obj.DelsysSensorsConnected)
                obj.DelsysComm.StartStreaming();
            end
        end
        function StartCapture(obj)
            % Start saving sensors data
            if(obj.DelsysSensorsConnected)
                obj.DelsysComm.StartCapture();
            end
        end
        function StartEMGplot(obj, display)
            % Start receiving EMG data for updating a plot
            if(obj.DelsysSensorsConnected)
                if(strcmp(display.Type, 'uitab'))
                    pos = [(display.Position(1)+display.Position(3)/2 + 95), (display.Position(2)+45), (display.Position(3)/2 -70), (display.Position(4)+5)];
                    display = figure('Name', 'EMG realTime', 'OuterPosition', pos, 'ToolBar', 'none', 'MenuBar', 'none', 'DockControls', 'off');
                else
                    display.Name = 'EMG realTime';
                end
                plotHandleEMG = obj.PlotEMG(1, display, 'realTime');
                obj.DelsysComm.StartEMGplot(plotHandleEMG);
            end
        end
        function StopEMGplot(obj)
            % Stop real time EMG plot
            if(obj.DelsysSensorsConnected)
                obj.DelsysComm.StopEMGplot();
                close 'EMG realTime';
            end
        end
        function EndReference(obj)
            % Stop sensor data capture and store values in DelsysReference
            if(obj.DelsysSensorsConnected)
                obj.DelsysComm.StopCapture();
                [~, ~, obj.EMG, obj.IMU, ~]  = obj.DelsysComm.GetCapture();
                obj.SaveReference();
            end
        end
        function EndTrial(obj, i_trial)
            % Stop sensor data capture and store values in DelsysTrial
            if(obj.DelsysSensorsConnected)
                obj.DelsysComm.StopCapture();
                [~, ~, obj.EMG, obj.IMU, ~]  = obj.DelsysComm.GetCapture();
                obj.SaveTrial(i_trial);
            end
        end
        function StopSensors(obj)
            % Stop sensors data streaming
            if(obj.DelsysSensorsConnected)
                obj.DelsysComm.StopStreaming();
            end
        end
        function CloseSensors(obj)
            % Disconnect sensors from user and delete network objects
            if(obj.DelsysSensorsConnected)
                obj.DelsysComm.CloseSession();
            end
        end
        
        % Plot functions
        function plotHandleEMG = PlotEMG(obj, arg, display, origin)
            % Plot recorded EMG signals for each sensor
            if (~exist('origin', 'var'))
                origin='results';
            end
            plotHandleEMG = 1;
            switch(origin)
                case 'results'
                    i_trial = arg;
                    if(obj.DelsysSensorsConnected)
                        obj.CalculateMinMeanMax(obj.DelsysTrial); %%ERROR
                        if i_trial>=1
                            DelsysSensors.PlotEMG_(obj.DelsysTrial{i_trial}, display);
                        else
                            obj.CalculateMinMeanMax(obj.DelsysTrial);
                            obj.PlotEMGrepeatibility(obj.DelsysTrial, display);
                        end
                    end
                case 'database'
                    delsysTrial = arg;
                    if(~isempty(delsysTrial))
                        if max(size(delsysTrial))==1
                            DelsysSensors.PlotEMG_(delsysTrial, display);
                        else
                            obj.CalculateMinMeanMax(delsysTrial);
                            obj.PlotEMGrepeatibility(delsysTrial, display);
                        end
                    else
                        ME = MException('DelsysSensors:PlotEMG', 'Empty DelsysSensors.Trial structure');
                        throw(ME);
                    end
                case 'realTime'
                    sgtitle(display, 'Señales EMG en tiempo real');
                    for i = 1:obj.NumSensors
                        ax(i) = subplot(obj.NumSensors, 1, i, 'Parent', display);
                        plotHandleEMG(i) = plot(ax(i),0,'-y','LineWidth',1);
                        set(ax(i),'YGrid','on');
                        set(ax(i),'XGrid','on');
                        set(ax(i),'Color',[.15 .15 .15]);
                        set(ax(i),'YLim', [-.00055 .00055]);
                        set(ax(i),'YLimMode', 'manual');
                        set(ax(i),'XLim', [0 15000]);
                        set(ax(i),'XLimMode', 'manual');
                        ylabel(ax(i),'V');
                        xlabel(ax(i),'Samples');
                        ylim(ax(i), [-0.00055, 0.00055]);
                        xlim(ax(i), [0 15000]);
                        title(ax(i), ['Sensor ', num2str(i), ': ', obj.SensorsMuscles{i}]);
                        drawnow;
                    end
                otherwise
                    ME=MException('DelsysSensors:PlotEMG', 'Wrong data origin specified. Must be "results" or "database"');
                    throw(ME);
            end
        end
        function PlotACC(obj, arg, display, origin)
            if (~exist('origin', 'var'))
                origin='results';
            end
            % Plot recorded acceleration signals (Accelerometer) for each sensor
            switch(origin)
                case 'results'
                    i_trial = arg;
                    if(obj.DelsysSensorsConnected)
                        obj.CalculateMinMeanMax(obj.DelsysTrial);
                        if i_trial>=1
                            DelsysSensors.PlotACC_(obj.DelsysTrial{i_trial}, display);
                        else
                            obj.CalculateMinMeanMax(obj.DelsysTrial);
                            obj.PlotACCrepeatibility(obj.DelsysTrial, display);
                        end
                    end
                case 'database'
                    delsysTrial = arg;
                    if(~isempty(delsysTrial))
                        if max(size(delsysTrial))==1
                            DelsysSensors.PlotACC_(delsysTrial, display);
                        else
                            obj.CalculateMinMeanMax(delsysTrial);
                            obj.PlotACCrepeatibility(delsysTrial, display);
                        end
                    else
                        ME = MException('DelsysSensors:PlotACC', 'Empty DelsysSensors.Trial structure');
                        throw(ME);
                    end
                otherwise
                    ME=MException('DelsysSensors:PlotACC', 'Wrong data origin specified. Must be "results" or "database"');
                    throw(ME);
            end
        end
        function PlotGYRO(obj, arg, display, origin)
            if (~exist('origin', 'var'))
                origin='results';
            end
            % Plot recorded angular velocity (gyroscope) signals for each sensor 
            switch(origin)
                case 'results'
                    i_trial = arg;
                    if(obj.DelsysSensorsConnected)
                        obj.CalculateMinMeanMax(obj.DelsysTrial);
                        if i_trial>=1
                            DelsysSensors.PlotGYRO_(obj.DelsysTrial{i_trial}, display);
                        else
                            obj.CalculateMinMeanMax(obj.DelsysTrial);
                            obj.PlotGYROrepeatibility(obj.DelsysTrial, display);
                        end
                    end
                case 'database'
                    delsysTrial = arg;
                    if(~isempty(delsysTrial))
                        if max(size(delsysTrial))==1
                            DelsysSensors.PlotGYRO_(delsysTrial, display);
                        else 
                            obj.CalculateMinMeanMax(delsysTrial);
                            obj.PlotGYROrepeatibility(delsysTrial, display);
                        end
                    else
                        ME = MException('DelsysSensors:PlotGYRO', 'Empty DelsysSensors.Trial structure');
                        throw(ME);
                    end
                otherwise
                    ME=MException('DelsysSensors:PlotGYRO', 'Wrong data origin specified. Must be "results" or "database"');
                    throw(ME);
            end
        end
        
        % Utilities
        function ClearTrials(obj)
            obj.DelsysTrial = [];
        end
        function delsysSensors = GetStruct(obj, i_trial)
            if(obj.DelsysSensorsConnected)
                delsysSensors.Reference = obj.DelsysReference;
                delsysSensors.Trial = obj.DelsysTrial{i_trial};
            else
                delsysSensors=[];
            end
        end
        function SetStruct(obj, DelsysTrials)
            for i = 1:size(DelsysTrials)
                obj.DelsysReference = DelsysTrials.DelsysSensors.Reference;
                obj.DelsysTrial{i,1} = Trials(idTrial(i)).DelsysSensors.Trial;
                obj.SetMovement(obj.DelsysReference.Sensor1.Movement);
            end
        end      
    end
    methods (Access = private)
        % Save and organize data in object properties
        function SaveReference(obj)
            % Save collected data
            for sensorId = 1:obj.NumSensors
                str = sprintf('Sensor%d',sensorId);
                sensorImuIdx = sensorId + 8*(sensorId - 1);

                % Save measured muscle name, emg timestamp and data, imu
                % timestamp,acc and gyro and sensor position

                obj.DelsysReference.(str).Movement = obj.Movement;
                obj.DelsysReference.(str).Muscle = obj.SensorsMuscles{sensorId};
                obj.DelsysReference.(str).Position = obj.SensorsPosition(sensorId,:);
                
                obj.DelsysReference.(str).ImuAll = double(obj.IMU.Values);
                obj.DelsysReference.(str).IMU.Timestamps = obj.IMU.Timestamps;
                obj.DelsysReference.(str).IMU.Acc = double(obj.IMU.Values(sensorImuIdx:sensorImuIdx+2,:)).*obj.gToMs2;
                obj.DelsysReference.(str).IMU.Gyro = deg2rad(double(obj.IMU.Values(sensorImuIdx+3:sensorImuIdx+5,:)));
                
                obj.DelsysReference.(str).EmgAll = double(obj.EMG.Values);
                obj.DelsysReference.(str).EMG.Timestamps = obj.EMG.Timestamps;
                obj.DelsysReference.(str).EMG.Raw = double(obj.EMG.Values(sensorId,:));
                obj.DelsysReference.(str).EMG.Filtered = DelsysSensors.emgFiltering(obj.DelsysReference.(str).EMG);
            end
        end
        function SaveTrial(obj, i_trial)
            % Save collected data         
            for sensorId = 1:obj.NumSensors
                str = sprintf('Sensor%d',sensorId);
                sensorImuIdx = sensorId + 8*(sensorId - 1);

                % Save measured muscle name, emg timestamp and data, imu
                % timestamp,acc and gyro and sensor position

                obj.DelsysTrial{i_trial}.(str).Movement = obj.Movement;
                obj.DelsysTrial{i_trial}.(str).Muscle = obj.SensorsMuscles{sensorId};
                obj.DelsysTrial{i_trial}.(str).Position = obj.SensorsPosition(sensorId,:);
                obj.DelsysTrial{i_trial}.(str).ImuAll = double(obj.IMU.Values);
                obj.DelsysTrial{i_trial}.(str).IMU.Timestamps = obj.IMU.Timestamps;
                obj.DelsysTrial{i_trial}.(str).IMU.Acc = double(obj.IMU.Values(sensorImuIdx:sensorImuIdx+2,:)).*obj.gToMs2;
                obj.DelsysTrial{i_trial}.(str).IMU.Gyro = deg2rad(double(obj.IMU.Values(sensorImuIdx+3:sensorImuIdx+5,:)));
                
                obj.DelsysTrial{i_trial}.(str).EmgAll = double(obj.EMG.Values);
                obj.DelsysTrial{i_trial}.(str).EMG.Timestamps = obj.EMG.Timestamps;
                obj.DelsysTrial{i_trial}.(str).EMG.Raw = double(obj.EMG.Values(sensorId,:));
                emgFiltered = DelsysSensors.emgFiltering(obj.DelsysTrial{i_trial}.(str).EMG);
                obj.DelsysTrial{i_trial}.(str).EMG.Filtered = emgFiltered - mean(obj.DelsysReference.(str).EMG.Filtered);
            end
        end
        
        % Plot functions
        function CalculateMinMeanMax(obj, DelsysTrial)
            numSensors = length(fieldnames(DelsysTrial{1}));
            DataUsed{size(DelsysTrial,2), numSensors} = [];
            nsamples = 10;
            for i = 1:size(DelsysTrial,2)
                for sensor = 1:numSensors
                    str = sprintf('Sensor%d', sensor);
                    DataUsed{i, sensor}.EMGrms.data = DelsysTrial{i}.(str).EMG.Filtered;
                    DataUsed{i, sensor}.EMGrms.t = DelsysTrial{i}.(str).EMG.Timestamps';
                    DataUsed{i, sensor}.acc.x = zeros(1, floor(size(DelsysTrial{i}.(str).IMU.Acc,2)/nsamples)+3); % +3 to include the last samples that don't necessarily form a pack and to include the first and last samples as they are
                    DataUsed{i, sensor}.acc.y = zeros(1, floor(size(DelsysTrial{i}.(str).IMU.Acc,2)/nsamples)+3);
                    DataUsed{i, sensor}.acc.z = zeros(1, floor(size(DelsysTrial{i}.(str).IMU.Acc,2)/nsamples)+3);
                    DataUsed{i, sensor}.gyro.x = zeros(1, floor(size(DelsysTrial{i}.(str).IMU.Acc,2)/nsamples)+3);
                    DataUsed{i, sensor}.gyro.y = zeros(1, floor(size(DelsysTrial{i}.(str).IMU.Acc,2)/nsamples)+3);
                    DataUsed{i, sensor}.gyro.z = zeros(1, floor(size(DelsysTrial{i}.(str).IMU.Acc,2)/nsamples)+3);
                    DataUsed{i, sensor}.tIMU = zeros(1, floor(size(DelsysTrial{i}.(str).IMU.Acc,2)/nsamples)+3);%obj.DelsysTrial{i,1}.(str).IMU.Timestamps(1:nsamples:end,:)';

                    for j = 1:nsamples:(size(DelsysTrial{i}.(str).IMU.Acc,2)-nsamples)
                        DataUsed{i, sensor}.acc.x(1,ceil(j/nsamples)+1) = mean(DelsysTrial{i}.(str).IMU.Acc(1,j:j+nsamples-1));
                        DataUsed{i, sensor}.acc.y(1,ceil(j/nsamples)+1) = mean(DelsysTrial{i}.(str).IMU.Acc(2,j:j+nsamples-1));
                        DataUsed{i, sensor}.acc.z(1,ceil(j/nsamples)+1) = mean(DelsysTrial{i}.(str).IMU.Acc(3,j:j+nsamples-1));
                        DataUsed{i, sensor}.gyro.x(1,ceil(j/nsamples)+1) = mean(DelsysTrial{i}.(str).IMU.Gyro(1,j:j+nsamples-1));
                        DataUsed{i, sensor}.gyro.y(1,ceil(j/nsamples)+1) = mean(DelsysTrial{i}.(str).IMU.Gyro(2,j:j+nsamples-1));
                        DataUsed{i, sensor}.gyro.z(1,ceil(j/nsamples)+1) = mean(DelsysTrial{i}.(str).IMU.Gyro(3,j:j+nsamples-1));
                        DataUsed{i, sensor}.tIMU(1,ceil(j/nsamples)+1) = mean(DelsysTrial{i}.(str).IMU.Timestamps(j:j+nsamples-1,1));
                    end

                    % Add last group of samples
                    j=j+nsamples;
                    DataUsed{i, sensor}.acc.x(1,ceil(j/nsamples)+1) = mean(DelsysTrial{i}.(str).IMU.Acc(1,j:end));
                    DataUsed{i, sensor}.acc.y(1,ceil(j/nsamples)+1) = mean(DelsysTrial{i}.(str).IMU.Acc(2,j:end));
                    DataUsed{i, sensor}.acc.z(1,ceil(j/nsamples)+1) = mean(DelsysTrial{i}.(str).IMU.Acc(3,j:end));
                    DataUsed{i, sensor}.gyro.x(1,ceil(j/nsamples)+1) = mean(DelsysTrial{i}.(str).IMU.Gyro(1,j:end));
                    DataUsed{i, sensor}.gyro.y(1,ceil(j/nsamples)+1) = mean(DelsysTrial{i}.(str).IMU.Gyro(2,j:end));
                    DataUsed{i, sensor}.gyro.z(1,ceil(j/nsamples)+1) = mean(DelsysTrial{i}.(str).IMU.Gyro(3,j:end));
                    DataUsed{i, sensor}.tIMU(1,ceil(j/nsamples)+1) = mean(DelsysTrial{i}.(str).IMU.Timestamps(j:end,1));

                    % Add first sample
                    DataUsed{i, sensor}.acc.x(1,1) = DelsysTrial{i}.(str).IMU.Acc(1,1);
                    DataUsed{i, sensor}.acc.y(1,1) = DelsysTrial{i}.(str).IMU.Acc(2,1);
                    DataUsed{i, sensor}.acc.z(1,1) = DelsysTrial{i}.(str).IMU.Acc(3,1);
                    DataUsed{i, sensor}.gyro.x(1,1) = DelsysTrial{i}.(str).IMU.Gyro(1,1);
                    DataUsed{i, sensor}.gyro.y(1,1) = DelsysTrial{i}.(str).IMU.Gyro(2,1);
                    DataUsed{i, sensor}.gyro.z(1,1) = DelsysTrial{i}.(str).IMU.Gyro(3,1);
                    DataUsed{i, sensor}.tIMU(1,1) = DelsysTrial{i}.(str).IMU.Timestamps(1,1);

                    % Add last sample
                    DataUsed{i, sensor}.acc.x(1,end) = DelsysTrial{i}.(str).IMU.Acc(1,end);
                    DataUsed{i, sensor}.acc.y(1,end) = DelsysTrial{i}.(str).IMU.Acc(2,end);
                    DataUsed{i, sensor}.acc.z(1,end) = DelsysTrial{i}.(str).IMU.Acc(3,end);
                    DataUsed{i, sensor}.gyro.x(1,end) = DelsysTrial{i}.(str).IMU.Gyro(1,end);
                    DataUsed{i, sensor}.gyro.y(1,end) = DelsysTrial{i}.(str).IMU.Gyro(2,end);
                    DataUsed{i, sensor}.gyro.z(1,end) = DelsysTrial{i}.(str).IMU.Gyro(3,end);
                    DataUsed{i, sensor}.tIMU(1,end) = DelsysTrial{i}.(str).IMU.Timestamps(end,1);
                end
            end

            sizesIMU = zeros(1,size(DataUsed,1));
            sizesEMG = zeros(1,size(DataUsed,1));

            for i = 1:size(DataUsed,1) %For each repetition
                sizesIMU(i) = size(DataUsed{i}.tIMU,2); %Check size of t for sensor1, assuming all sensors from a trial have the same t
                sizesEMG(i) = size(DataUsed{i}.EMGrms.t,2); %Check size of t
            end

            min_sizeIMU = min(sizesIMU);
            min_sizeIMU = min_sizeIMU - 1;
            tsampleIMU = mean(DataUsed{1}.tIMU(2:end)-DataUsed{1}.tIMU(1:end-1)); % Assuming all trials and sensors have the same tsample
            obj.DataMean.tIMU = linspace(0, min_sizeIMU*tsampleIMU, min_sizeIMU);
            obj.DataMin.tIMU = obj.DataMean.tIMU;
            obj.DataMax.tIMU = obj.DataMean.tIMU;

            min_sizeEMG = min(sizesEMG);
            min_sizeEMG = min_sizeEMG - 1;
            tsampleEMG = mean(DataUsed{1}.EMGrms.t(2:end)-DataUsed{1}.EMGrms.t(1:end-1));
            obj.DataMean.EMGrms.t = linspace(0, min_sizeEMG*tsampleEMG, min_sizeEMG);
            obj.DataMin.EMGrms.t = obj.DataMean.EMGrms.t;
            obj.DataMax.EMGrms.t = obj.DataMean.EMGrms.t;

            obj.DataMean.EMGrms.data = zeros(numSensors, min_sizeEMG);
            obj.DataMin.EMGrms.data = zeros(numSensors, min_sizeEMG);
            obj.DataMax.EMGrms.data = zeros(numSensors, min_sizeEMG);

            obj.DataMean.acc.x = zeros(numSensors, min_sizeIMU);
            obj.DataMin.acc.x = zeros(numSensors, min_sizeIMU);
            obj.DataMax.acc.x = zeros(numSensors, min_sizeIMU);
            obj.DataMean.acc.y = zeros(numSensors, min_sizeIMU);
            obj.DataMin.acc.y = zeros(numSensors, min_sizeIMU);
            obj.DataMax.acc.y = zeros(numSensors, min_sizeIMU);
            obj.DataMean.acc.z = zeros(numSensors, min_sizeIMU);
            obj.DataMin.acc.z = zeros(numSensors, min_sizeIMU);
            obj.DataMax.acc.z = zeros(numSensors, min_sizeIMU);

            obj.DataMean.gyro.x = zeros(numSensors, min_sizeIMU);
            obj.DataMin.gyro.x = zeros(numSensors, min_sizeIMU);
            obj.DataMax.gyro.x = zeros(numSensors, min_sizeIMU);
            obj.DataMean.gyro.y = zeros(numSensors, min_sizeIMU);
            obj.DataMin.gyro.y = zeros(numSensors, min_sizeIMU);
            obj.DataMax.gyro.y = zeros(numSensors, min_sizeIMU);
            obj.DataMean.gyro.z = zeros(numSensors, min_sizeIMU);
            obj.DataMin.gyro.z = zeros(numSensors, min_sizeIMU);
            obj.DataMax.gyro.z = zeros(numSensors, min_sizeIMU);

            emg = zeros(size(DataUsed,1), min_sizeEMG);

            for sensor = 1:numSensors
                for i = 1:size(DataUsed,1)
                    acc.x(i,:) = DataUsed{i, sensor}.acc.x(1, 1:min_sizeIMU);
                    acc.y(i,:) = DataUsed{i, sensor}.acc.y(1, 1:min_sizeIMU);
                    acc.z(i,:) = DataUsed{i, sensor}.acc.z(1, 1:min_sizeIMU);

                    gyro.x(i,:) = DataUsed{i, sensor}.gyro.x(1, 1:min_sizeIMU);
                    gyro.y(i,:) = DataUsed{i, sensor}.gyro.y(1, 1:min_sizeIMU);
                    gyro.z(i,:) = DataUsed{i, sensor}.gyro.z(1, 1:min_sizeIMU);

                    emg(i,:) = DataUsed{i, sensor}.EMGrms.data(1, 1:min_sizeEMG);
                end

                obj.DataMean.acc.x(sensor,:) = mean(acc.x,1);
                obj.DataMin.acc.x(sensor,:) = min(acc.x,[],1);
                obj.DataMax.acc.x(sensor,:) = max(acc.x,[],1);
                obj.DataMean.acc.y(sensor,:) = mean(acc.y,1);
                obj.DataMin.acc.y(sensor,:) = min(acc.y,[],1);
                obj.DataMax.acc.y(sensor,:) = max(acc.y,[],1);
                obj.DataMean.acc.z(sensor,:) = mean(acc.z,1);
                obj.DataMin.acc.z(sensor,:) = min(acc.z,[],1);
                obj.DataMax.acc.z(sensor,:) = max(acc.z,[],1);

                obj.DataMean.gyro.x(sensor,:) = mean(gyro.x,1);
                obj.DataMin.gyro.x(sensor,:) = min(gyro.x,[],1);
                obj.DataMax.gyro.x(sensor,:) = max(gyro.x,[],1);
                obj.DataMean.gyro.y(sensor,:) = mean(gyro.y,1);
                obj.DataMin.gyro.y(sensor,:) = min(gyro.y,[],1);
                obj.DataMax.gyro.y(sensor,:) = max(gyro.y,[],1);
                obj.DataMean.gyro.z(sensor,:) = mean(gyro.z,1);
                obj.DataMin.gyro.z(sensor,:) = min(gyro.z,[],1);
                obj.DataMax.gyro.z(sensor,:) = max(gyro.z,[],1);

                obj.DataMean.EMGrms.data(sensor,:) = mean(emg,1);
                obj.DataMin.EMGrms.data(sensor,:) = min(emg,[],1);
                obj.DataMax.EMGrms.data(sensor,:) = max(emg,[],1);
            end
        end  
        function PlotEMGrepeatibility(obj, DelsysTrial, display)
            numSensors = length(fieldnames(DelsysTrial{1}));
%             figure('WindowState', 'maximized', 'Name', 'Repetibilidad EMG')
%             hold on
            ax = zeros(1,numSensors);
            x = [obj.DataMean.EMGrms.t, fliplr(obj.DataMean.EMGrms.t)];
            for i = 1:numSensors
                str = sprintf('Sensor%d', i);
                ax(i) = subplot(numSensors, 1, i, 'Parent', display);
                hold(ax(i), 'on');
                y=[obj.DataMax.EMGrms.data(i,:), fliplr(obj.DataMin.EMGrms.data(i,:))];
                fill(ax(i), x,y,obj.ColorLightBlue);
                plot(ax(i), obj.DataMean.EMGrms.t, obj.DataMax.EMGrms.data(i,:), 'b');
                plot(ax(i), obj.DataMean.EMGrms.t, obj.DataMin.EMGrms.data(i,:), 'b');
                plot(ax(i), obj.DataMean.EMGrms.t, obj.DataMean.EMGrms.data(i,:), 'b', 'LineWidth', 1);
                xlabel(ax(i), 'Tiempo [s]')
                ylabel(ax(i), '$EMG [V]$')
                title(ax(i), [str, ': ', DelsysTrial{1}.(str).Muscle], 'FontSize', 12)
            end
            sgtitle(display, 'Repetibilidad EMG');
        end
        function PlotACCrepeatibility(obj, DelsysTrial, display)
            numSensors = length(fieldnames(DelsysTrial{1}));
            sgtitle(display, 'Repetibilidad aceleración');
%             figure('WindowState', 'maximized', 'Name', 'Repetibilidad Aceleraciones')
%             hold on
%             ax = zeros(1,numSensors*3);
            x = [obj.DataMean.tIMU, fliplr(obj.DataMean.tIMU)];
            for i = 1:numSensors
                str = sprintf('Sensor%d', i);
                ax(1+3*(i-1)) = subplot(numSensors, 3, 1+3*(i-1), 'Parent', display);
                hold(ax(1+3*(i-1)), 'on');
                y=[obj.DataMax.acc.x(i,:), fliplr(obj.DataMin.acc.x(i,:))];
                fill(ax(1+3*(i-1)), x,y,obj.ColorLightRed);
                plot(ax(1+3*(i-1)), obj.DataMean.tIMU, obj.DataMax.acc.x(i,:), 'r');
                plot(ax(1+3*(i-1)), obj.DataMean.tIMU, obj.DataMin.acc.x(i,:), 'r');
                plot(ax(1+3*(i-1)), obj.DataMean.tIMU, obj.DataMean.acc.x(i,:), 'r', 'LineWidth', 1);
                xlabel(ax(1+3*(i-1)), 'Tiempo [s]')
                ylabel(ax(1+3*(i-1)), '$\dot{XYZ} [m/s^{2}]$')
                title(ax(1), {'Eje X', ''}, 'FontSize', 13)
                if(strcmp(display.Type, 'uitab'))
                    drawnow;
                    w = display.Position(3)*(ax(1+3*(i-1)).OuterPosition(3)/3);
                    h = display.Position(4)*(ax(1+3*(i-1)).Position(4)*2/3);
                    xpos = (display.Position(3)*ax(1+3*(i-1)).OuterPosition(1))-w*3/2.8;
                    ypos = display.Position(2)+display.Position(4)*(ax(1+3*(i-1)).Position(2)+ ax(1+3*(i-1)).Position(4)/2) - h/2;
                    pos = [xpos, ypos, w, h];
                    uilabel(display, 'Text', {['Sensor ',num2str(i),': '], DelsysTrial{1}.(str).Muscle}, 'Position', pos, 'FontSize', 15, ...
                        'HorizontalAlignment', 'center', 'BackgroundColor', obj.ColorLightBlue);
                else
                    display.WindowState = 'maximized';
                    drawnow;
                    w = ax(1+3*(i-1)).Position(3)/3;
                    h = ax(1+3*(i-1)).Position(4)*2/3;
                    xpos = ax(1+3*(i-1)).OuterPosition(1)-w*3/2.8;
                    ypos = ax(1+3*(i-1)).Position(2)+ ax(1+3*(i-1)).Position(4)/2 - h/2;
                    pos = [xpos, ypos, w, h];
                    annotation('textbox', pos, 'String', {['Sensor ',num2str(i),': '], DelsysTrial{1}.(str).Muscle}, 'FontSize', 14, ...
                        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'BackgroundColor', obj.ColorLightBlue,...
                        'FaceAlpha', 0.5)
                end
                
                ax(2+3*(i-1)) = subplot(numSensors, 3, 2+3*(i-1), 'Parent', display);
                hold(ax(2+3*(i-1)), 'on');
                y=[obj.DataMax.acc.y(i,:), fliplr(obj.DataMin.acc.y(i,:))];
                fill(ax(2+3*(i-1)), x,y,obj.ColorLightBlue);
                plot(ax(2+3*(i-1)), obj.DataMean.tIMU, obj.DataMax.acc.y(i,:), 'b');
                plot(ax(2+3*(i-1)), obj.DataMean.tIMU, obj.DataMin.acc.y(i,:), 'b');
                plot(ax(2+3*(i-1)), obj.DataMean.tIMU, obj.DataMean.acc.y(i,:), 'b', 'LineWidth', 1);
                xlabel(ax(2+3*(i-1)), 'Tiempo [s]')
                ylabel(ax(2+3*(i-1)), '$\dot{XYZ} [m/s^{2}]$')
                title(ax(2), {'Eje Y', ''}, 'FontSize', 13)
                
                ax(3+3*(i-1)) = subplot(numSensors, 3, 3+3*(i-1), 'Parent', display);
                hold(ax(3+3*(i-1)), 'on');
                y=[obj.DataMax.acc.z(i,:), fliplr(obj.DataMin.acc.z(i,:))];
                fill(ax(3+3*(i-1)), x,y,obj.ColorLightGreen);
                plot(ax(3+3*(i-1)), obj.DataMean.tIMU, obj.DataMax.acc.z(i,:), 'g');
                plot(ax(3+3*(i-1)), obj.DataMean.tIMU, obj.DataMin.acc.z(i,:), 'g');
                plot(ax(3+3*(i-1)), obj.DataMean.tIMU, obj.DataMean.acc.z(i,:), 'g', 'LineWidth', 1);
                xlabel(ax(3+3*(i-1)), 'Tiempo [s]')
                ylabel(ax(3+3*(i-1)), '$\dot{XYZ} [m/s^{2}]$')
                title(ax(3), {'Eje Z', ''}, 'FontSize', 13)
            end
        end
        function PlotGYROrepeatibility(obj, DelsysTrial, display)
            numSensors = length(fieldnames(DelsysTrial{1}));
%             figure('WindowState', 'maximized', 'Name', 'Repetibilidad Velocidades angulares')
%             hold on
            sgtitle(display, 'Repetibilidad velocidad angular');
            x = [obj.DataMean.tIMU, fliplr(obj.DataMean.tIMU)];
            for i = 1:numSensors
                str = sprintf('Sensor%d', i);
                ax(1+3*(i-1)) = subplot(numSensors, 3, 1+3*(i-1), 'Parent', display);
%                 ax(1+3*(i-1)).Units = 'pixels';
                hold(ax(1+3*(i-1)), 'on');
                y=[obj.DataMax.gyro.x(i,:), fliplr(obj.DataMin.gyro.x(i,:))];
                fill(ax(1+3*(i-1)), x,y,obj.ColorLightRed);
                plot(ax(1+3*(i-1)), obj.DataMean.tIMU, obj.DataMax.gyro.x(i,:), 'r');
                plot(ax(1+3*(i-1)), obj.DataMean.tIMU, obj.DataMin.gyro.x(i,:), 'r');
                plot(ax(1+3*(i-1)), obj.DataMean.tIMU, obj.DataMean.gyro.x(i,:), 'r', 'LineWidth', 1);
                xlabel(ax(1+3*(i-1)), 'Tiempo [s]')
                ylabel(ax(1+3*(i-1)), '$\omega [rad/s]$')
                title(ax(1), {'Eje X', ''}, 'FontSize', 13);
                if(strcmp(display.Type, 'uitab'))
                    drawnow;
                    w = display.Position(3)*(ax(1+3*(i-1)).OuterPosition(3)/3);
                    h = display.Position(4)*(ax(1+3*(i-1)).Position(4)*2/3);
                    xpos = (display.Position(3)*ax(1+3*(i-1)).OuterPosition(1))-w*3/2.8;
                    ypos = display.Position(2)+display.Position(4)*(ax(1+3*(i-1)).Position(2)+ ax(1+3*(i-1)).Position(4)/2) - h/2;
                    pos = [xpos, ypos, w, h];
                    uilabel(display, 'Text', {['Sensor ',num2str(i),': '], DelsysTrial{1}.(str).Muscle}, 'Position', pos, 'FontSize', 15, ...
                        'HorizontalAlignment', 'center', 'BackgroundColor', obj.ColorLightBlue);
                else
                    display.WindowState = 'maximized';
                    drawnow;
                    w = ax(1+3*(i-1)).Position(3)/3;
                    h = ax(1+3*(i-1)).Position(4)*2/3;
                    xpos = ax(1+3*(i-1)).OuterPosition(1)-w*3/2.8;
                    ypos = ax(1+3*(i-1)).Position(2)+ ax(1+3*(i-1)).Position(4)/2 - h/2;
                    pos = [xpos, ypos, w, h];
                    annotation('textbox', pos, 'String', {['Sensor ',num2str(i),': '], DelsysTrial{1}.(str).Muscle}, 'FontSize', 14, ...
                        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'BackgroundColor', obj.ColorLightBlue,...
                        'FaceAlpha', 0.5)
                end
                
                ax(2+3*(i-1)) = subplot(numSensors, 3, 2+3*(i-1), 'Parent', display);
%                 ax(2+3*(i-1)).Units = 'pixels';
                hold(ax(2+3*(i-1)), 'on');
                y=[obj.DataMax.gyro.y(i,:), fliplr(obj.DataMin.gyro.y(i,:))];
                fill(ax(2+3*(i-1)), x,y,obj.ColorLightBlue);
                plot(ax(2+3*(i-1)), obj.DataMean.tIMU, obj.DataMax.gyro.y(i,:), 'b');
                plot(ax(2+3*(i-1)), obj.DataMean.tIMU, obj.DataMin.gyro.y(i,:), 'b');
                plot(ax(2+3*(i-1)), obj.DataMean.tIMU, obj.DataMean.gyro.y(i,:), 'b', 'LineWidth', 1);
                xlabel(ax(2+3*(i-1)), 'Tiempo [s]')
                ylabel(ax(2+3*(i-1)), '$\omega [rad/s]$')
                title(ax(2), {'Eje Y', ''}, 'FontSize', 13)
                
                ax(3+3*(i-1)) = subplot(numSensors, 3, 3+3*(i-1), 'Parent', display);
%                 ax(3+3*(i-1)).Units = 'pixels';
                hold(ax(3+3*(i-1)), 'on');
                y=[obj.DataMax.gyro.z(i,:), fliplr(obj.DataMin.gyro.z(i,:))];
                fill(ax(3+3*(i-1)), x,y,obj.ColorLightGreen);
                plot(ax(3+3*(i-1)), obj.DataMean.tIMU, obj.DataMax.gyro.z(i,:), 'g');
                plot(ax(3+3*(i-1)), obj.DataMean.tIMU, obj.DataMin.gyro.z(i,:), 'g');
                plot(ax(3+3*(i-1)), obj.DataMean.tIMU, obj.DataMean.gyro.z(i,:), 'g', 'LineWidth', 1);
                xlabel(ax(3+3*(i-1)), 'Tiempo [s]')
                ylabel(ax(3+3*(i-1)), '$\omega [rad/s]$')
                title(ax(3), {'Eje Z', ''}, 'FontSize', 13);    
            end
        end
    end
    methods (Static)
        function PlotEMG_(DelsysTrial, display)
            numSensors = length(fieldnames(DelsysTrial));
            ax = zeros(1,numSensors);
            for i = 1:numSensors
                str = sprintf('Sensor%d', i);
                emg_noOffset = DelsysTrial.(str).EMG.Raw - mean(DelsysTrial.(str).EMG.Raw);
%                 hold on
                ax(i) = subplot(numSensors, 1, i, 'Parent', display);
                hold(ax(i), 'on');
                plot(ax(i), DelsysTrial.(str).EMG.Timestamps,emg_noOffset, ...
                    'Color', [0.56 0.75 0.87], 'LineWidth', 0.35)
                plot(ax(i), DelsysTrial.(str).EMG.Timestamps, DelsysTrial.(str).EMG.Filtered, 'LineWidth', 2, 'Color', 'r')
                xlabel(ax(i), 'Tiempo [s]')
                ylabel(ax(i), '$EMG [V]$')
                legend(ax(i), 'EMG capturada', 'RMS EMG')
%                     title([str, ': ', SensorsMuscles{i}], 'FontSize', 12)
                title(ax(i), [str, ': ', DelsysTrial.(str).Muscle], 'FontSize', 12)
            end
            sgtitle(display, 'EMG');
        end
        function PlotACC_(DelsysTrial, display)
            numSensors = length(fieldnames(DelsysTrial));
            ax = zeros(1,numSensors);
%             figure('WindowState', 'maximized', 'Name', 'Aceleraciones')
            for i = 1:numSensors
                str = sprintf('Sensor%d', i);
%                 hold on
                ax(i) = subplot(numSensors, 1, i, 'Parent', display);
                hold(ax(i), 'on');
                plot(ax(i), DelsysTrial.(str).IMU.Timestamps, DelsysTrial.(str).IMU.Acc(1,:), 'Color', 'r')
                plot(ax(i), DelsysTrial.(str).IMU.Timestamps, DelsysTrial.(str).IMU.Acc(2,:), 'Color', 'b')
                plot(ax(i), DelsysTrial.(str).IMU.Timestamps, DelsysTrial.(str).IMU.Acc(3,:), 'Color', 'g')
                xlabel(ax(i), 'Tiempo [s]')
                ylabel(ax(i), '$\dot{XYZ} [m/s^{2}]$')
                legend(ax(i), 'X', 'Y', 'Z')
                title(ax(i), [str, ': ', DelsysTrial.(str).Muscle], 'FontSize', 12)
            end
            sgtitle(display, 'Aceleración');
        end
        function PlotGYRO_(DelsysTrial, display)
            numSensors = length(fieldnames(DelsysTrial));
            ax = zeros(1,numSensors);
%             figure('WindowState', 'maximized', 'Name', 'Velocidades Angulares')
            for i = 1:numSensors
                str = sprintf('Sensor%d', i);
%                 hold on
                ax(i) = subplot(numSensors, 1, i, 'Parent', display);
                hold(ax(i), 'on');
                plot(ax(i), DelsysTrial.(str).IMU.Timestamps, DelsysTrial.(str).IMU.Gyro(1,:), 'Color', 'r')
                plot(ax(i), DelsysTrial.(str).IMU.Timestamps, DelsysTrial.(str).IMU.Gyro(2,:), 'Color', 'b')
                plot(ax(i), DelsysTrial.(str).IMU.Timestamps, DelsysTrial.(str).IMU.Gyro(3,:), 'Color', 'g')
                legend(ax(i), 'X', 'Y', 'Z')
                xlabel(ax(i), 'Tiempo [s]')
                ylabel(ax(i), '$\omega [rad/s]$')
                title(ax(i), [str, ': ', DelsysTrial.(str).Muscle], 'FontSize', 12)
            end
            sgtitle(display, 'Velocidad angular');
        end
        function emgFiltered = emgFiltering(emg)
            % Obtain the filtered EMG signal
            % Signal frequency
            Fs = 1/(emg.Timestamps(2)-emg.Timestamps(1));

            % Remove offset from signal
            emg_noOffset = emg.Raw - mean(emg.Raw);
            
            % Bandpass butterworth filter of order 2, with cutoff f = [30,300]
            % Correct f with filter order to ensure that fcut is the desired
            fcut = [30, 300]./(sqrt(2)-1)^(0.5/2);
            wn = 2*(1/Fs).*fcut;
            [b,a] = butter(2, wn, 'bandpass');
            emg_filtered = filtfilt(b,a,emg_noOffset);

            % Rectify filtered signal
            emg_rectified = abs(emg_filtered);
            
            % Lowpass butterworth filter of order 2, with cutoff freq 6 to
            % obtain the EMG linear envelope
            fcut = 6/(sqrt(2)-1)^(0.5/2);
            wn = 2*(1/Fs)*fcut;
            [b,a] = butter(2, wn, 'low');
            emgFiltered = filtfilt(b, a, emg_rectified);
        end 
%         function [emg_RMS, time_RMS] = emgFiltering(emg)
%             % Obtain the filtered RMS EMG signal
%             % Signal frequency
%             Fs = 1/(emg.Timestamps(2)-emg.Timestamps(1));
% 
%             % Bandpass butterworth filter of order 4 (8), with cutoff f = [10,450]
%             [b,a] = butter(4, [10 450]/Fs, 'bandpass');
%             emg_filtered = filter(b,a,emg.Data);
% 
%             % RMS of the filtered signal 50% overlap
%             emg_RMS = DelsysSensors.rms_delsys(emg_filtered,100,50,1)';
% 
%             time_RMS = linspace(0,emg.Timestamps(end,1), size(emg_RMS,1))';
%         end
%         function y = rms_delsys(signal, windowlength, overlap, zeropad)
%             %% DECLARATIONS AND INITIALIZATIONS
%             % Calculates windowed (over- and non-overlapping) RMS of a signal using the specified windowlength
%             % y = rms(signal, windowlength, overlap, zeropad)
%             % signal is a 1-D vector
%             % windowlength is an integer length of the RMS window in samples
%             % overlap is the number of samples to overlap adjacent windows (enter 0 to use non-overlapping windows)
%             % zeropad is a flag for zero padding the end of your data...(0 for NO, 1 for YES)
%             % ex. y=rms(mysignal, 30, 10, 1).  Calculate RMS with window of length 30 samples, overlapped by 10 samples each, and zeropad the last window if necessary
%             % ex. y=rms(mysignal, 30, 0, 0).  Calculate RMS with window of length 30 samples, no overlapping samples, and do not zeropad the last window
%             %
%             % Author: A. Bolu Ajiboye
% 
%             delta = windowlength - overlap;
%             % CALCULATE RMS
%             indices = 1:delta:length(signal);
%             % Zeropad signal
%             if length(signal) - indices(end) + 1 < windowlength
%                 if zeropad
%                     signal(end+1:indices(end)+windowlength-1) = 0;
%                 else
%                     indices = indices(1:find(indices+windowlength-1 <= length(signal), 1, 'last'));
%                 end
%             end
%             y = zeros(1, length(indices));
%             % Square the samples
%             signal = signal.^2;
%             index = 0;
%             for i = indices
%                 index = index+1;
%                 % Average and take the square root of each window
%                 y(index) = sqrt(mean(signal(i:i+windowlength-1)));
%             end
%         end
    end
end