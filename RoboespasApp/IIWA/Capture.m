classdef Capture < handle
    properties
        StartClient
        StopClient
        ClearClient
        ListClient
        SaveClient
        SaveMsg
        NewClient
        NewMsg
        LoadClient
        LoadMsg
    end
    methods
        function obj = Capture()
            obj.InitServices();
        end     
        function InitServices(obj)
            obj.StartClient = rossvcclient('/roboespas/capture/start');
            obj.StopClient = rossvcclient('/roboespas/capture/stop');
            obj.ClearClient = rossvcclient('/roboespas/capture/clear');
            obj.ListClient  = rossvcclient('/roboespas/capture/list');
            [obj.SaveClient, obj.SaveMsg]=rossvcclient('roboespas/capture/save');
            [obj.NewClient, obj.NewMsg]=rossvcclient('roboespas/capture/new');
            [obj.LoadClient, obj.LoadMsg]=rossvcclient('roboespas/capture/load');
        end
        function Start(obj)
            obj.Stop();
            obj.Clear();
            call(obj.StartClient);
        end
        function Stop(obj)
            call(obj.StopClient);
        end
        function Clear(obj)
            call(obj.ClearClient);
        end
        function List = List(obj, IncludedString)
            response=call(obj.ListClient);
            List={};
            n=1;
            for i=1:size(response.Names,1)
                if(~isempty(strfind(response.Names{i}, IncludedString)))
                    List{n}=response.Names{i}; %TODO
                    n=n+1;
                end
            end
        end
        function LoadData = Load(obj, Name, Folder)
            obj.LoadMsg.Name=Name;
            obj.LoadMsg.Folder=Folder;
            load_response = obj.LoadClient.call(obj.LoadMsg);
            if (~isempty(load_response.TKnot))
                % If the interpolation polynomials are present, calculate the joint
                % trajectory, velocity trajectory, ... with the polynomials
                %disp('Found interpolation polynomials')
                LoadData.pp.breaks=load_response.TKnot';
                LoadData.t=load_response.T';
                nSeg=size(LoadData.pp.breaks,2)-1;
                LoadData.pp.coefs.q=permute(reshape(load_response.QCoefs, 4, 7, nSeg), [1 3 2]);
                LoadData.pp.coefs.qdot=permute(reshape(load_response.QdotCoefs, 3, 7, nSeg), [1 3 2]);
                LoadData.pp.coefs.qdotdot=permute(reshape(load_response.QdotdotCoefs, 2, 7, nSeg), [1 3 2]);
                LoadData=get_data_from_pp(LoadData);
            else     
                % If the interpolation polynomials weren't found, fill with the
                % information found in the jointPosition.txt, ... files
                %disp('Didnt find interpolation polynomials')
                nPoint=size(load_response.Q,1)/7;
                LoadData.t=load_response.T';
                LoadData.q=reshape(load_response.Q, 7, nPoint);
                x=reshape(load_response.X, 6, nPoint);
                LoadData.x.pos=x(1:3, :);
                LoadData.x.ori=x(4:6, :);
                LoadData.qdot=reshape(load_response.Qdot, 7, nPoint);
                xdot=reshape(load_response.Xdot, 6, nPoint);
                LoadData.xdot.pos=xdot(1:3, :);
                LoadData.xdot.ori=xdot(4:6, :);
                if (exist('LoadData', 'var')==0)
                    LoadData.q=zeros(7,1);
                    LoadData.t=0;
                    LoadData.qdot=zeros(7,1);
                end
                LoadData.qdotdot=[(LoadData.qdot(:, 2:end)-LoadData.qdot(:, 1:end-1))./(LoadData.t(2:end)-LoadData.t(1:end-1)), zeros(7,1)];
                if (~isempty(LoadData.t))
                    LoadData.t=LoadData.t-LoadData.t(1);
                    LoadData=fill_cartesian(LoadData);
                else
                    disp('Couldnt find an INET address'); %TODO: Why?
                end
            end
            if (~isempty(load_response.Ttorque))
                nTorques=size(load_response.Qtorque,1)/7;
                LoadData.t_torque=load_response.Ttorque';
                LoadData.q_torque=reshape(load_response.Qtorque, 7, nTorques);
            end
        end
        function Save(obj, name, folder)
            obj.SaveMsg.Name=name;
            obj.SaveMsg.Folder=[folder, '_captured'];
            obj.SaveClient.call(obj.SaveMsg);
            obj.Clear();
        end
        function Data = SaveAndLoad(obj, name, folder)
            obj.Save(name, folder);
            Data = obj.Load(name, [folder, '_captured']);
        end
        function New(obj, name, folder, Data)
            format long;
            % Read data
            obj.NewMsg.Name=name;
            obj.NewMsg.Folder=folder;
            obj.NewMsg.T=Data.t;
            obj.NewMsg.Q=reshape(Data.q, 1, size(Data.q,1)*size(Data.q,2));
            obj.NewMsg.Qdot=reshape(Data.qdot, 1, size(Data.qdot,1)*size(Data.qdot,2));
            x=[Data.x.pos; Data.x.ori];
            obj.NewMsg.X=reshape(x, 1, size(x,1)*size(x,2));
            if (isfield(Data, 'xdot'))
                xdot=[Data.xdot.pos; Data.xdot.ori];
                obj.NewMsg.Xdot=reshape(xdot, 1, size(xdot,1)*size(xdot,2));
            else
                obj.NewMsg.Xdot=[];
            end
            if isfield(Data, 'pp')
                obj.NewMsg.TKnot=Data.pp.breaks;
                qcoefs=permute(Data.pp.coefs.q, [1 3 2]);
                qdotcoefs=permute(Data.pp.coefs.qdot, [1 3 2]);
                qdotdotcoefs=permute(Data.pp.coefs.qdotdot, [1 3 2]);
                obj.NewMsg.QCoefs=reshape(qcoefs, 1, size(qcoefs,1)*size(qcoefs,2)*size(qcoefs,3), 1);
                obj.NewMsg.QdotCoefs=reshape(qdotcoefs, 1, size(qdotcoefs,1)*size(qdotcoefs,2)*size(qdotcoefs,3), 1);
                obj.NewMsg.QdotdotCoefs=reshape(qdotdotcoefs, 1, size(qdotdotcoefs,1)*size(qdotdotcoefs,2)*size(qdotdotcoefs,3), 1);
            else
                obj.NewMsg.TKnot=[];
                obj.NewMsg.QCoefs=[];
                obj.NewMsg.QdotCoefs=[];
                obj.NewMsg.QdotdotCoefs=[];
            end
            obj.NewClient.call(obj.NewMsg);
        end
    end
end

