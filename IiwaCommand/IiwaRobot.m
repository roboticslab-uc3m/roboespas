classdef IiwaRobot
    %IIWAROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        l_tool = 0.152; %ManetaFT
        n_joints = 7;
        pk=[0;0;0.36]; 
        pf=[0;0;1.18]; 
        pp=[0;0; 1.306 + IiwaRobot.l_tool];
        Twist = [0   -0.3600         0    0.7800         0   -1.1800         0;
                 0         0         0         0         0         0         0;
                 0         0         0         0         0         0         0;
                 0         0         0         0         0         0         0;
                 0    1.0000         0   -1.0000         0    1.0000         0;
            1.0000         0    1.0000         0    1.0000         0     1.000];
        Hst0 = [1 0 0 0; 0 1 0 0; 0 0 1 1.306 + IiwaRobot.l_tool; 0 0 0 1];
        % Maximum Magnitude for the robot joints POSITION rad, (by catalog).
        Thmax = deg2rad([170 120 170 120 170 120 175]);
        ThDotmax = deg2rad([75 75 90 90 144 135 135]);
        Torquemax = [176 176 110 110 110 40 40]; %Nm
        CartAccMax = 0.5 %m/s2
    end
    
    methods
        function obj = IiwaRobot()
        end
    end
end

