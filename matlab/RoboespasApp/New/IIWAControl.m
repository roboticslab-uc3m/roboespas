classdef IIWAControl < handle
    properties
        MasterIP=find_ip(); %Get this computer's IP
        MasterPort=11311;
    end
    properties(Access = private)
        FRIConnected = 0
        GazeboConnected = 0
    end
    
    methods
        function obj = IIWAControl()
        end
    end
end

