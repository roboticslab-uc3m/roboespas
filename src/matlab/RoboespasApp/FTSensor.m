classdef FTSensor < handle
    properties(Constant)
        %Simulink filename
        SimulinkFilename='ReadGagesLong';
        %Sensor Data
        Serial='FT17951';
        NumGages=6;
        Family='DAQ';
        BodyStyle='Delta'
        PartNumber='SI-660-60'
        CalibrationDate='12/04/2015'
        ForceUnits='N'
        TorqueUnits='Nm'
        DistUnits='m'
        OutputMode='Ground Referenced Differential'
        OutputRange=20
        HWTempComp=1
        GainMultiplier=1
        CableLossDetection=0
        OutputBipolar=1
        %Calibration matrix
        MWithLegend=["0","gage_1","gage_2","gage_3","gage_4","gage_5","gage_6";"Fx","-1.55665","0.19951","-1.67131","-70.01985","1.18367","69.2685";"Fy","4.13533","80.7128","-3.32482","-40.53408","1.36578","-40.08142";"Fz","119.8679","3.95338","119.1754","3.17747","120.6012","-0.93021";"Tx","0.04403","0.97135","-4.19451","-0.59969","4.16689","-0.50782";"Ty","4.77034","0.15205","-2.42076","0.77813","-2.3512","-0.81726";"Tz","-0.12562","-2.62475","-0.08604","-2.65289","-0.03934","-2.63343"];
        M=[-1.55665000000000,0.199510000000000,-1.67131000000000,-70.0198500000000,1.18367000000000,69.2685000000000;4.13533000000000,80.7128000000000,-3.32482000000000,-40.5340800000000,1.36578000000000,-40.0814200000000;119.867900000000,3.95338000000000,119.175370000000,3.17747000000000,120.601230000000,-0.930210000000000;0.0440300000000000,0.971350000000000,-4.19451000000000,-0.599690000000000,4.16689000000000,-0.507820000000000;4.77034000000000,0.152050000000000,-2.42076000000000,0.778130000000000,-2.35120000000000,-0.817260000000000;-0.125620000000000,-2.62475000000000,-0.0860400000000000,-2.65289000000000,-0.0393400000000000,-2.63343000000000]
        %Other non used constants present in Calibration File
        MScaled=[-0.775489044214791,0.0993915261692049,-0.832610152883835,-34.8823605515453,0.589678551354332,34.5080543855023;2.06013112081248,40.2093547861752,-1.65635273438873,-20.1931936898634,0.680401777411542,-19.9676883606280;19.7351264875892,0.650886971019808,19.6211079125874,0.523140660347932,19.8558624002659,-0.153150359771217;0.364378007583337,8.03857773486428,-34.7124051008036,-4.96284004923123,34.4838309338844,-4.20255370908404;39.4777875243038,1.25831651267423,-20.0334250655789,6.43955164753173,-19.4577690535985,-6.76337884346032;-0.939602183905937,-19.6323900032408,-0.643554942710292,-19.8428692697200,-0.294252108858936,-19.6973139560851] %Not used
        R=[1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0;0,0,0,1,0,0;0,0,0,0,1,0;0,0,0,0,0,1];
        D=[1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0;0,0.0120142000000000,0,1,0,0;-0.0120142000000000,0,0,0,1,0;0,0,0,0,0,1];
        MaxVal=[660; 660; 1980; 60; 60; 60]
        BasicTransform=[0; 0; 0.120; 0; 0; 0]
        scale=[0.498178167356047;0.498178167356047;0.164640629289319;8.27567584790681;8.27567584790681;7.47971806962217]
        %Other constants
        MaxFiles=10;
        
        ColorLightBlue = [152/256, 163/256, 243/256];   
        PathSaveCaptures = 'G:\Mi unidad\ROBOESPAS_COLABORATIVE\RoboespasMatlab\Develop\Common\FTSensor_Tmp\gages_simout_'
    end
    properties(Access=private)
        FTSensorConnected
        PathMatrix
        
        GagesReferenceCaptured
        GagesTrialCaptured
        GagesReference
        GagesTrial
        ForceTorqueTrial
        Started;
        
        StartTimeTrial
        StartTimeReference
        EndTimeTrial
        EndTimeReference
        
        AllGages
        
        % Repeatibility variables
        DataMin
        DataMean
        DataMax
    end
    methods (Access=public)
        function obj = FTSensor(ConnectFT)
            if (ConnectFT)
                %TODO: Open Simulink
                obj.PathMatrix = obj.BuildPossiblePaths();
                obj.FTSensorConnected = 1;
            else
                obj.FTSensorConnected = 0;
            end
        end
        function StartSensor(obj)
            if (obj.FTSensorConnected)
                obj.ClearFiles();
                obj.StartTimeReference=[];
                obj.EndTimeReference=[];
                obj.StartTimeTrial=[];
                obj.EndTimeTrial=[];
                set_param(obj.SimulinkFilename, 'SimulationCommand', 'start');
                pause(0.1);
                %Wait until capture has really started
                while (obj.ReadTimeSimulink<0.5)
                    pause(0.01);
                end
            end
        end
        function StartReference(obj)
            if (obj.FTSensorConnected)
                obj.StartTimeReference = obj.ReadTimeSimulink();
            end
        end
        function StartTrial(obj, i_trial)
            if (obj.FTSensorConnected)
                obj.StartTimeTrial{i_trial} = obj.ReadTimeSimulink();
            end
        end
        function EndReference(obj)
            if (obj.FTSensorConnected)
                obj.EndTimeReference = obj.ReadTimeSimulink();
            end
        end
        function EndTrial(obj, i_trial)
            if (obj.FTSensorConnected)
                obj.EndTimeTrial{i_trial} = obj.ReadTimeSimulink();
            end
        end
        function StopSensor(obj)
            if (obj.FTSensorConnected)
                set_param(obj.SimulinkFilename, 'SimulationCommand', 'stop')
                while(~strcmp(get_param(obj.SimulinkFilename, 'SimulationStatus'), 'stopped'))
                    pause(0.05);
                end
                obj.ReadGagesFromFiles();
                obj.ClearFiles();
            end
        end
        function GetReference(obj)
            if (obj.FTSensorConnected)
                obj.GagesReferenceCaptured = obj.GetGagesFromAllGages(obj.StartTimeReference, obj.EndTimeReference);
            end
        end
        function FTSensor = GetStruct(obj, i_trial)
            if (obj.FTSensorConnected)
                obj.GetTrial(i_trial);
                obj.CalculateForceTorque(i_trial);
                FTSensor.GagesReference = obj.GagesReference;
                FTSensor.GagesTrial = obj.GagesTrial{i_trial};
                FTSensor.MUsed = obj.M;
                FTSensor.ForceTorqueTrial = obj.ForceTorqueTrial{i_trial};
            else
                FTSensor=[];
            end
        end
        function PlotFT(obj, origin, arg, display)
            switch(origin)
                case 'results'
                    i_trial = arg;
                    if (obj.FTSensorConnected)
                        if i_trial>=1
                            FTSensor.PlotFT_(obj.ForceTorqueTrial{i_trial}, display);
                        else
                            obj.CalculateMinMeanMax(obj.ForceTorqueTrial);
                            obj.PlotFTrepeatibility(display);
                        end
                    end
                case 'database'
                    forceTorqueTrial = arg;
                    if(~isempty(forceTorqueTrial))
                        if max(size(forceTorqueTrial))==1
                            FTSensor.PlotFT_(forceTorqueTrial, display);
                        else
                            obj.CalculateMinMeanMax(forceTorqueTrial);
                            obj.PlotFTrepeatibility(display);
                        end
                    else
                        ME = MException('FTSensor:PlotFT', 'Empty FTSensor.ForceTorqueTrial structure');
                        throw(ME);
                    end
            end
        end
        function CalculateMinMeanMax(obj, ForceTorqueTrial)
            DataUsed{size(ForceTorqueTrial,2)} = [];
            nsamples = 1;
            for i = 1:size(ForceTorqueTrial,2)
                DataUsed{i}.force = zeros(3, floor(size(ForceTorqueTrial{i}.Values,1)/nsamples)+3); % +3 to include the last samples that don't necessarily form a pack and to include the first and last samples as they are
                DataUsed{i}.torque = zeros(3, floor(size(ForceTorqueTrial{i}.Values,1)/nsamples)+3);
                DataUsed{i}.t = zeros(1, floor(size(ForceTorqueTrial{i}.Timestamps,1)/nsamples)+3);

                for j = 1:nsamples:(size(ForceTorqueTrial{i}.Values,1)-nsamples)
                    DataUsed{i}.force(:,ceil(j/nsamples)+1) = mean(ForceTorqueTrial{i}.Values(j:j+nsamples-1,1:3),1)';
                    DataUsed{i}.torque(:,ceil(j/nsamples)+1) = mean(ForceTorqueTrial{i}.Values(j:j+nsamples-1,4:6),1)';
                    DataUsed{i}.t(1,ceil(j/nsamples)+1) = mean(ForceTorqueTrial{i}.Timestamps(j:j+nsamples-1));
                end

                % Add last group of samples
                j=j+nsamples;
                DataUsed{i}.force(:,ceil(j/nsamples)+1) = mean(ForceTorqueTrial{i}.Values(j:end,1:3),1)';
                DataUsed{i}.torque(:,ceil(j/nsamples)+1) = mean(ForceTorqueTrial{i}.Values(j:end,4:6),1)';
                DataUsed{i}.t(1,ceil(j/nsamples)+1) = mean(ForceTorqueTrial{i}.Timestamps(j:end));

                % Add first sample
                DataUsed{i}.force(:,1) = ForceTorqueTrial{i}.Values(1,1:3)';
                DataUsed{i}.torque(:,1) = ForceTorqueTrial{i}.Values(1,4:6)';
                DataUsed{i}.t(1,1) = ForceTorqueTrial{i}.Timestamps(1);

                % Add last sample
                DataUsed{i}.force(:,end) = ForceTorqueTrial{i}.Values(end,1:3)';
                DataUsed{i}.torque(:,end) = ForceTorqueTrial{i}.Values(end,4:6)';
                DataUsed{i}.t(1,end) = ForceTorqueTrial{i}.Timestamps(end);
            end
            sizes = zeros(1,size(DataUsed,2));

            for i = 1:size(DataUsed,2) %For each repetition
                sizes(i) = size(DataUsed{i}.t,2); % Check size of t
            end

            min_size = min(sizes);
            size_used = min_size - 1;
            tsample = mean(DataUsed{1}.t(2:end)-DataUsed{1}.t(1:end-1)); % Assuming all trials have the same tsample
            obj.DataMean.t = linspace(0, size_used*tsample, size_used);
            obj.DataMin.t = obj.DataMean.t;
            obj.DataMax.t = obj.DataMean.t;

            obj.DataMean.force = zeros(3, size_used);
            obj.DataMin.force = zeros(3, size_used);
            obj.DataMax.force = zeros(3, size_used);
            obj.DataMean.torque = zeros(3, size_used);
            obj.DataMin.torque = zeros(3, size_used);
            obj.DataMax.torque = zeros(3, size_used);

            force = zeros(size(DataUsed,2), size_used);
            torque = zeros(size(DataUsed,2), size_used);

            for coord = 1:3
                for i = 1:size(DataUsed,2)
                    force(i,:) = DataUsed{i}.force(coord, 1:size_used);
                    torque(i,:) = DataUsed{i}.torque(coord, 1:size_used);
                end

                obj.DataMean.force(coord,:) = mean(force,1);
                obj.DataMin.force(coord,:) = min(force,[],1);
                obj.DataMax.force(coord,:) = max(force,[],1);
                obj.DataMean.torque(coord,:) = mean(torque,1);
                obj.DataMin.torque(coord,:) = min(torque,[],1);
                obj.DataMax.torque(coord,:) = max(torque,[],1);
            end
        end
    end
    methods (Access=private)
        function PlotFTrepeatibility(obj, display)
%             figure('WindowState', 'maximized', 'Name', 'Repetibilidad Sensor Fuerza Par')
%             hold on
            sgtitle(display, 'Repetibilidad sensor Fuerza-Par');
            x = [obj.DataMean.t, fliplr(obj.DataMean.t)];
            coordName = ['X', 'Y', 'Z'];
            ax = zeros(1,6);
            for coord = 1:3
                ax(coord) = subplot(2, 3, coord, 'Parent', display);
                hold(ax(coord), 'on');
                y=[obj.DataMax.force(coord,:), fliplr(obj.DataMin.force(coord,:))];
                fill(ax(coord), x,y,obj.ColorLightBlue);
                plot(ax(coord), obj.DataMean.t, obj.DataMax.force(coord,:), 'b');
                plot(ax(coord), obj.DataMean.t, obj.DataMin.force(coord,:), 'b');
                plot(ax(coord), obj.DataMean.t, obj.DataMean.force(coord,:), 'b', 'LineWidth', 1);
                xlabel(ax(coord), 'Tiempo [s]')
                ylabel(ax(coord), 'Fuerza [N]')
                title(ax(coord), ['Fuerza en ', coordName(coord)])
                
                ax(coord+3) = subplot(2, 3, coord+3, 'Parent', display);
                hold(ax(coord+3), 'on');
                y=[obj.DataMax.torque(coord,:), fliplr(obj.DataMin.torque(coord,:))];
                fill(ax(coord+3), x,y,obj.ColorLightBlue);
                plot(ax(coord+3), obj.DataMean.t, obj.DataMax.torque(coord,:), 'b');
                plot(ax(coord+3), obj.DataMean.t, obj.DataMin.torque(coord,:), 'b');
                plot(ax(coord+3), obj.DataMean.t, obj.DataMean.torque(coord,:), 'b', 'LineWidth', 1);
                xlabel(ax(coord+3), 'Tiempo [s]')
                ylabel(ax(coord+3), 'Par [Nm]')
                title(ax(coord+3), ['Par en ', coordName(coord)])
            end
        end
        function PathMatrix = BuildPossiblePaths(obj)
            for i=obj.MaxFiles-1:-1:0
                PathMatrix{i+1, :}=strcat(obj.PathSaveCaptures, num2str(i), '.mat');
            end
        end
        function ReadGagesFromFiles(obj)
            gages=[];
            t=[];
            for i=10:-1:1
                if (exist(obj.PathMatrix{i}, 'file'))
                    load(obj.PathMatrix{i});
                    gages=[gages_simout.signals(1).values; gages];
                    t=[floor(gages_simout.time*1e2)/1e2 t];
                end
            end
            obj.AllGages.Values=gages;
            obj.AllGages.Timestamps=t';
        end
        function Gages = GetGagesFromAllGages(obj, tstart, tend)            
            %t=0:round(mean(t(2:end)-t(1:end-1)),2):t(end);
            %disp(num2str(tend-tstart));
            idstart = find(abs(obj.AllGages.Timestamps-tstart)<1e-2, 1, 'first');
            idend = find(abs(obj.AllGages.Timestamps-tend)<1e-2, 1, 'first');
            trial_timestamps=obj.AllGages.Timestamps(idstart:idend);
            trial_timestamps=trial_timestamps-trial_timestamps(1);
            trial_gages=obj.AllGages.Values(idstart:idend, :);
            Gages.Values=trial_gages;
            Gages.Timestamps=trial_timestamps;
            %disp(num2str(Gages.Timestamps(end)));
        end
        function GagesFiltered = FilterGages(obj, GagesCaptured)
            SampleTimeTrial = mean(GagesCaptured.Timestamps(2:end) - GagesCaptured.Timestamps(1:end-1));
            GagesFiltered.Values = zeros(size(GagesCaptured.Values));
            for i=1:obj.NumGages
                GagesFiltered.Values(:,i) = lowpass(GagesCaptured.Values(:,i), 0.5, 1/SampleTimeTrial);
            end
            GagesFiltered.Timestamps = GagesCaptured.Timestamps;
        end
        function CalculateForceTorque(obj, i_trial)
            if (obj.FTSensorConnected)
                %Filter both GagesTrialCaptured{i_trial} and
                %GagesReferenceCaptured
                GagesTrialFiltered = obj.FilterGages(obj.GagesTrialCaptured{i_trial});
                GagesRefFiltered = obj.FilterGages(obj.GagesReferenceCaptured);
                %Take the same amount of points for both of them
                MinSize = min([size(GagesTrialFiltered.Values,1), size(GagesRefFiltered.Values,1)]);
                obj.GagesTrial{i_trial}.Values = GagesTrialFiltered.Values(1:MinSize, :);%1:obj.NumGages);
                obj.GagesTrial{i_trial}.Timestamps = GagesTrialFiltered.Timestamps(:, 1:MinSize);
                obj.GagesReference.Values = GagesRefFiltered.Values(1:MinSize, :);%1:obj.NumGages);
                obj.GagesReference.Timestamps = GagesRefFiltered.Timestamps(:, 1:MinSize);
                %Calculate difference and multiply by M
                GagesDifference = obj.GagesTrial{i_trial}.Values - obj.GagesReference.Values;
                obj.ForceTorqueTrial{i_trial}.Values = zeros(MinSize, obj.NumGages);
                for i=1:MinSize
                    obj.ForceTorqueTrial{i_trial}.Values(i,:)=obj.M*GagesDifference(i,:)';
                end
                obj.ForceTorqueTrial{i_trial}.Timestamps=obj.GagesTrial{i_trial}.Timestamps;
            end
        end
        function ClearFiles(obj)
            for i=obj.MaxFiles:-1:1
                if (exist(obj.PathMatrix{i}, 'file'))
                    delete(obj.PathMatrix{i});
                end
            end
        end
        function time = ReadTimeSimulink(obj)
            time = get_param(obj.SimulinkFilename, 'SimulationTime');
        end
        function GetTrial(obj, i_trial)
            obj.GagesTrialCaptured{i_trial} = obj.GetGagesFromAllGages(obj.StartTimeTrial{i_trial}, obj.EndTimeTrial{i_trial});
        end
    end
    methods (Static, Access = private)
        function PlotFT_(ForceTorqueTrial, display)
%             figure('WindowState', 'maximized', 'Name', 'Sensor fuerza par');
            sgtitle(display, 'Sensor Fuerza-Par');
            ax(1) = subplot(1,2,1, 'Parent', display);
            plot(ax(1), ForceTorqueTrial.Timestamps, ForceTorqueTrial.Values(:,1:3),'DisplayName','FTvals')    
            legend(ax(1), {'fx', 'fy', 'fz'});
            xlabel(ax(1), 'Tiempo [s]');
            ylabel(ax(1), 'Fuerza [N]');
            ax(2) = subplot(1,2,2, 'Parent', display);
            plot(ax(2), ForceTorqueTrial.Timestamps, ForceTorqueTrial.Values(:,4:6),'DisplayName','FTvals')    
            legend(ax(2), {'tx', 'ty', 'tz'});
            xlabel(ax(2), 'Tiempo [s]');
            ylabel(ax(2), 'Par [Nm]');
        end
    end
end

