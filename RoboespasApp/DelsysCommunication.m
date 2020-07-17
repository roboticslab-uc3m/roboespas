classdef DelsysCommunication < handle
    %DELSYSCOMMUNICATION 
    properties (Access = private)
        %Communication with sensors properties
        CommObject
        InterfaceObjectEMG
        InterfaceObjectIMU
        HostIP
        BytePackageSizeEMG %960
        BytePackageSizeIMU %1152
        BufferSizeIMU %57600 -> Stores a maximum of 50 packages of size 1152
        BufferSizeEMG %69120 -> Stores a maximum of 72 packages of size 960
        TimerReadEMGIMU
        TimerPlotEMG
        
        %Communication with workers properties
        WithParallelWorkers = 0;
        DataQueueClientToWorkerConstant
        DataQueueClientToWorker
        DataQueueWorkerToClient
        DataQueueWorkerToClientEMG
        
        %Flags to communicate with timer
        CaptureFlag = 0
        StopCaptureFlag = 0
        
        %Debug variables
        StopwatchFread; %Stopwatch that will measure time passing between fread functions
        
        %Display variables
        PlotListener
    end
    properties (Access = public)
        %Usually debugged variables
        BinaryCellIMU = {}
        BinaryCellEMG = {} 
        TimesFread =[];
    end
    properties (Constant, Access = private)
        %Communication with sensors properties
        HostPortEMG = 50043
        HostPortIMU = 50044
        HostPortComm = 50040
        Timeout = 0.5
        MaxSensors = 16;
        ChannelsIMU = 9;
        EMGPerFrame = 15;
        IMUPerFrame = 2;
        FramePeriod = 0.0135;
    end
    methods (Access = public)
        function obj = DelsysCommunication(bParallel)
            % Class constructor to enable communication with parallel pool.
            % The class is not in parallel.
            obj.WithParallelWorkers=bParallel;

            obj.BytePackageSizeEMG=4*obj.EMGPerFrame*obj.MaxSensors;
            obj.BytePackageSizeIMU=4*obj.IMUPerFrame*obj.MaxSensors*obj.ChannelsIMU;
            obj.BufferSizeEMG=60*obj.BytePackageSizeEMG;
            obj.BufferSizeIMU=60*obj.BytePackageSizeIMU;
            % TCPIP Connection to stream EMG Data
            obj.HostIP = char(java.net.InetAddress.getLocalHost.getHostAddress);
            obj.InterfaceObjectEMG = tcpip(obj.HostIP,obj.HostPortEMG);
            obj.InterfaceObjectEMG.InputBufferSize = obj.BufferSizeEMG;

            % TCPIP Connection to stream IMU Data
            obj.InterfaceObjectIMU = tcpip(obj.HostIP, obj.HostPortIMU);
            obj.InterfaceObjectIMU.InputBufferSize = obj.BufferSizeIMU;

            % TCPIP Connection to communicate with SDK, send/receive commands
            obj.CommObject = tcpip(obj.HostIP, obj.HostPortComm);
            obj.CommObject.Timeout = obj.Timeout;

            % Open communication
            if (obj.WithParallelWorkers==1)
                % Parallel pool communication configuration
                % Creation of a Constant PollableDataQueue for the communication from
                % client (main program) -> worker (parallel pool)
                obj.DataQueueClientToWorkerConstant = parallel.pool.Constant(@parallel.pool.PollableDataQueue);

                % Get the worker to send back the value of the
                % PollableDataQueueConstant to confirm communication between
                % both. If parpool didn't exist, it is created when parfeval is first
                % called
                obj.DataQueueClientToWorker = fetchOutputs(parfeval(@obj.returnQueue, ...
                    1, obj.DataQueueClientToWorkerConstant));

                % Create PollableDataQueue to enable communication from
                % worker(parallel pool) -> client (main program)
                obj.DataQueueWorkerToClient = parallel.pool.PollableDataQueue;
                
                % Create DataQueue to enable communication from worker
                % (parallel pool) -> client (main program) for realtime EMG streaming
                obj.DataQueueWorkerToClientEMG = parallel.pool.DataQueue;
                parfeval(@infiniteLoop, 0, obj);
            else
                obj.initSession();
            end
        end           
        % Sensors status control methods
        function [binEMG, binIMU, EMG, IMU, timesFread] = GetCapture(obj)
            if (obj.WithParallelWorkers==1)
                send(obj.DataQueueClientToWorker, 'getCapture');
                data=obj.waitForMessage();
                binEMG = data{1, 1};
                binIMU = data{1, 2};
                EMG = data{1, 3};
                IMU = data{1, 4};
                timesFread = data{1, 5};
            else
                [binEMG, binIMU, EMG, IMU, timesFread] = obj.getCapture();
            end
        end
        function StopCapture(obj)
        	if (obj.WithParallelWorkers==1)
                send(obj.DataQueueClientToWorker, 'stopCapture');
                obj.waitForMessage();
        	else
                obj.stopCapture();
        	end 
        end
        function StartCapture(obj)
            if (obj.WithParallelWorkers==1)
                send(obj.DataQueueClientToWorker, 'startCapture');
                obj.waitForMessage();
            else
                obj.startCapture();
            end 
        end
        function StartEMGplot(obj, display)
            if (obj.WithParallelWorkers==1)
                send(obj.DataQueueClientToWorker, 'startStreaming');
                obj.waitForMessage();
                obj.PlotListener = afterEach(obj.DataQueueWorkerToClientEMG, @(data) updatePlot(display,data));
                send(obj.DataQueueClientToWorker, 'startEMGplot');
            else
                obj.startStreaming();
                obj.startCapture();
            end 
        end
        function StopEMGplot(obj)
            if (obj.WithParallelWorkers==1)
                send(obj.DataQueueClientToWorker, 'stopEMGplot');
%                 obj.waitForMessage();
                send(obj.DataQueueClientToWorker, 'stopStreaming');
                obj.waitForMessage();
                delete(obj.PlotListener);
            else
                obj.stopCapture();
                obj.stopStreaming();
                obj.BinaryCellEMG = {};
                obj.BinaryCellIMU = {};
                obj.TimesFread = [];    
            end
        end
        function StartStreaming(obj)
            %Must wait 3 seconds after calling the function (it's non-blocking)
            if (obj.WithParallelWorkers==1)
                send(obj.DataQueueClientToWorker, 'startStreaming');
                obj.waitForMessage();
            else
                obj.startStreaming();
            end
        end
        function StopStreaming(obj)
            if (obj.WithParallelWorkers==1)
                send(obj.DataQueueClientToWorker, 'stopStreaming');
                obj.waitForMessage();
            else
                obj.stopStreaming();
            end
            
        end
        function CloseSession(obj)
            if (obj.WithParallelWorkers)
                try
                    send(obj.DataQueueClientToWorker, 'closeSession');
                catch ex
                    disp('Session not closed because no parallel pool running');
                end
            else
                obj.closeSession();
            end
        end
    end
    methods (Access = public)
        function [arrayEMG, arrayIMU] = reshapeBinArrays(obj, binEMG, binIMU)
            BinEMGarray=cell2mat(binEMG);
            BinIMUarray=cell2mat(binIMU);
            
            EMGarray=typecast(cast(BinEMGarray, 'uint8'), 'single');
            IMUarray=typecast(cast(BinIMUarray, 'uint8'), 'single');
            
            ReshapedEMG=reshape(EMGarray, obj.MaxSensors, size(EMGarray,1)/obj.MaxSensors);
            ReshapedIMU=reshape(IMUarray, obj.MaxSensors*obj.ChannelsIMU, size(IMUarray,1)/(obj.MaxSensors*obj.ChannelsIMU));

            PeriodEMG=obj.FramePeriod/obj.EMGPerFrame;
            PeriodIMU=obj.FramePeriod/obj.IMUPerFrame;

            TimestampEMG = (0:PeriodEMG:(size(ReshapedEMG,2)-1)*PeriodEMG)';
            TimestampIMU = (0:PeriodIMU:(size(ReshapedIMU,2)-1)*PeriodIMU)';
            
            %Calculate nFrames
            TimestampFrameIMU = (0:obj.FramePeriod:(size(ReshapedIMU,2)-1)*obj.FramePeriod/obj.IMUPerFrame)';
            TimestampFrameEMG = (0:obj.FramePeriod:(size(ReshapedEMG,2)-1)*obj.FramePeriod/obj.EMGPerFrame)';
            nFrames=min([size(TimestampFrameIMU,1), size(TimestampFrameEMG,1)]);

            %Crop
            Error=1e-5;
            CropEMG=find(abs(TimestampEMG-TimestampFrameEMG(nFrames))<Error, 1, 'last');
            CropIMU=find(abs(TimestampIMU-TimestampFrameIMU(nFrames))<Error, 1, 'last');

            arrayEMG.Values=ReshapedEMG(:,1:CropEMG);
            arrayIMU.Values=ReshapedIMU(:,1:CropIMU);

            arrayEMG.Timestamps=TimestampEMG(1:CropEMG);
            arrayIMU.Timestamps=TimestampIMU(1:CropIMU); 
        end
        function answer = waitForMessage(obj)
            timer=tic;
            t=toc(timer);
            twait=15;
            while t<twait
                [data, gotMsg] = poll(obj.DataQueueWorkerToClient);
                if (gotMsg)
                    answer=data;
                    return;
                end
                t=toc(timer);
            end
            if t>=twait
                ME=MException('DelsysCommunication:Error', 'Error en el parallel pool. Reinicie el parallel pool, el programa de Trigno y la interfaz.');
                throw(ME);
            end
            answer='Error';
        end
        function initSession(obj)
            % Open and configure the sensors communication object
            fopen(obj.CommObject);
            fprintf(obj.CommObject, sprintf('UPSAMPLE OFF\r\n\r'));
            fprintf(obj.CommObject, sprintf('SENSOR 1 SETMODE 23\r\n\r'));
            fprintf(obj.CommObject, sprintf('SENSOR 2 SETMODE 23\r\n\r'));
            fprintf(obj.CommObject, sprintf('SENSOR 3 SETMODE 23\r\n\r'));
            fprintf(obj.CommObject, sprintf('SENSOR 4 SETMODE 23\r\n\r'));
            fprintf(obj.CommObject, sprintf('SENSOR 5 SETMODE 23\r\n\r'));
            fprintf(obj.CommObject, sprintf('SENSOR 6 SETMODE 23\r\n\r'));
            fopen(obj.InterfaceObjectEMG);
            fopen(obj.InterfaceObjectIMU);
            % Timer configuration
            obj.TimerReadEMGIMU=timer('TimerFcn', {@readEMGIMU, obj}, ...
                'ExecutionMode', 'fixedRate', ...
                'Period', 0.01);
            obj.TimerPlotEMG=timer('TimerFcn', {@sendEMG, obj}, ...
                'ExecutionMode', 'fixedRate', ...
                'Period', 0.1);
            pause(0.5);
        end
        function infiniteLoop(obj) 
            obj.initSession();
            while true
                % Wait for a message
                pause(0.01);
                [state, gotMsg] = poll(obj.DataQueueClientToWorker);
                if(gotMsg)
                    if (~isempty(state))
                        switch state
                            case 'stopCapture'
                                obj.stopCapture();
                                send(obj.DataQueueWorkerToClient, {1});
                            case 'getCapture'
                                [binEMG, binIMU, EMG, IMU, timesFread] = obj.getCapture();
                                send(obj.DataQueueWorkerToClient, {binEMG, binIMU, EMG, IMU, timesFread});
                            case 'startCapture'
                                obj.startCapture();
                                send(obj.DataQueueWorkerToClient, {1});
                            case 'startEMGplot'
                                obj.startCapture();
                                start(obj.TimerPlotEMG);
                            case 'stopEMGplot'
                                stop(obj.TimerPlotEMG);
                                obj.stopCapture();
                                obj.BinaryCellEMG = {};
                                obj.BinaryCellIMU = {};
                                obj.TimesFread = [];
%                                 send(obj.DataQueueWorkerToClient, {1});
                            case 'startStreaming'
                                obj.startStreaming();
                                send(obj.DataQueueWorkerToClient, {1});
                            case 'stopStreaming'
                                obj.stopStreaming();
                                send(obj.DataQueueWorkerToClient, {1});
                            case 'closeSession'
                                %%TODO:Hacer bloqueante
                                obj.closeSession();
                                break;
                        end
                    end
                end
            end
        end
        function [binEMG, binIMU, EMG, IMU, timesFread] = getCapture(obj)
            binEMG = obj.BinaryCellEMG;
            binIMU = obj.BinaryCellIMU;
            timesFread = obj.TimesFread;
            [EMG, IMU] = obj.reshapeBinArrays(binEMG, binIMU);
            obj.BinaryCellEMG = {};
            obj.BinaryCellIMU = {};
            obj.TimesFread = [];
        end
        function stopCapture(obj)
            obj.StopCaptureFlag=1;
        end
        function startCapture(obj)
            flushinput(obj.InterfaceObjectIMU);
            flushinput(obj.InterfaceObjectEMG);
            obj.BinaryCellEMG = {};
            obj.BinaryCellIMU = {};
            obj.TimesFread = [];  
            obj.CaptureFlag=1;
            obj.StopwatchFread=tic;
        end
        function closeSession(obj) 
            stop(obj.TimerReadEMGIMU);
            delete(obj.TimerReadEMGIMU);
            stop(obj.TimerPlotEMG);
            delete(obj.TimerPlotEMG);
            % Clean up the network objects
            if isvalid(obj.InterfaceObjectEMG)
                fclose(obj.InterfaceObjectEMG);
                delete(obj.InterfaceObjectEMG);
                clear obj.InterfaceObjectEMG;
            end

            if isvalid(obj.InterfaceObjectIMU)
                fclose(obj.InterfaceObjectIMU);
                delete(obj.InterfaceObjectIMU);
                clear obj.InterfaceObjectIMU;
            end

            if isvalid(obj.CommObject)
                fclose(obj.CommObject);
                delete(obj.CommObject);
                clear obj.CommObject;
            end
        end
        function startStreaming(obj)
            fprintf(obj.CommObject, sprintf('START\r\n\r'));
            timer_startstreaming=tic;
            while (obj.InterfaceObjectEMG.BytesAvailable==0 || obj.InterfaceObjectIMU.BytesAvailable==0)
                if (toc(timer_startstreaming)>10)
                    fprintf(obj.CommObject, sprintf('START\r\n\r'));
                    timer_startstreaming=tic;
                    disp('Resent start streaming');
                end
                pause(0.1);
            end
            start(obj.TimerReadEMGIMU);
            pause(0.5);
        end
        function stopStreaming(obj)
            obj.stopCapture();
            stop(obj.TimerReadEMGIMU);
            fprintf(obj.CommObject, sprintf('STOP\r\n\r'));
            timer_stopstreaming=tic;
            while (obj.InterfaceObjectEMG.BytesAvailable>0 || obj.InterfaceObjectIMU.BytesAvailable>0)
                if (toc(timer_stopstreaming)>10)
                    fprintf(obj.CommObject, sprintf('STOP\r\n\r'));
                    timer_stopstreaming=tic;
                    disp('Resent stop streaming');
                end
                flushinput(obj.InterfaceObjectEMG);
                flushinput(obj.InterfaceObjectIMU);
                pause(0.1);
            end
            pause(3); %%?
        end
    end
    methods (Static)
        function DataQueueClientToWorker = returnQueue(DataQueueClientToWorkerConstant)
            DataQueueClientToWorker = DataQueueClientToWorkerConstant.Value;
        end
    end
end

function readEMGIMU(~, ~, obj)
    %timer_temp=tic;
    if (obj.CaptureFlag==1 && obj.InterfaceObjectEMG.BytesAvailable>=10*obj.BytePackageSizeEMG)
        disp('Flush buffer EMG')
        flushinput(obj.InterfaceObjectEMG);
    end
    if (obj.CaptureFlag==1 && obj.InterfaceObjectIMU.BytesAvailable>=10*obj.BytePackageSizeIMU)
        disp('Flush buffer IMU')
        flushinput(obj.InterfaceObjectIMU);
    end
    
    bytesAvailableEMG=obj.InterfaceObjectEMG.BytesAvailable;
    bytesAvailableIMU=obj.InterfaceObjectIMU.BytesAvailable; 
    
    if ((obj.CaptureFlag ||obj.StopCaptureFlag) && bytesAvailableEMG>0 && bytesAvailableIMU>0)
        obj.TimesFread = [obj.TimesFread; toc(obj.StopwatchFread)];
        obj.StopwatchFread=tic;
        binEMG=fread(obj.InterfaceObjectEMG, bytesAvailableEMG);
        n=size(obj.BinaryCellEMG,1);
        obj.BinaryCellEMG{n+1, 1} = binEMG;    
        binIMU=fread(obj.InterfaceObjectIMU, bytesAvailableIMU);
        n=size(obj.BinaryCellIMU,1);
        obj.BinaryCellIMU{n+1, 1}=binIMU;
        if (obj.StopCaptureFlag)
            obj.StopCaptureFlag=0;
            obj.CaptureFlag=0;
        end
    end
    %timer_dur=toc(timer_temp);
    %disp(['Timer duration:', num2str(timer_dur)]);
end

function sendEMG(~, ~, obj)
    data_array = cell2mat(obj.BinaryCellEMG);
    data_uint = cast(data_array, 'uint8');
    data_single = typecast(data_uint, 'single');
    data_reshaped = reshape(data_single, obj.MaxSensors, size(data_single,1)/obj.MaxSensors);
    if(size(data_reshaped,2)>(15000)) && (mod(size(data_reshaped,2),(15000))>10)
        data_reshaped = data_reshaped(:,(end-(15000)):end);
    end
    send(obj.DataQueueWorkerToClientEMG, data_reshaped);
end

function updatePlot(display, data)
%     data_array = cell2mat(data);
%     data_uint = cast(data_array, 'uint8');
%     data_single = typecast(data_uint, 'single');
%     data_reshaped = reshape(data, 16, size(data,1)/16);
%     if(size(data_reshaped,2)>(15000)) && (mod(size(data_reshaped,2),(15000))>10)
%         data_reshaped = data_reshaped(:,(end-(15000)):end);
%     end
    for i = 1:max(size(display))
        y_data = data(i,:);
%         if(size(y_data,1)>15000) && (mod(size(y_data,1),15000)>10)
%             y_data = y_data((end-15000):end,1);
%         end
        x_data = 1:size(y_data,2);
        set(display(i), 'YData', y_data);
        set(display(i), 'XData', x_data);
        drawnow;
    end
%     drawnow;
end