classdef IiwaRobot
    %IIWAROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant, Access = 'public')
        PoAcc = [0 0 -9.81]'; %Potential action vector - gravity
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
        LiMas = [IiwaRobot.CM1 IiwaRobot.CM2 IiwaRobot.CM3 IiwaRobot.CM4 ...
            IiwaRobot.CM5 IiwaRobot.CM6 IiwaRobot.CM7;
            IiwaRobot.IT1 IiwaRobot.IT2 IiwaRobot.IT3 IiwaRobot.IT4 ...
            IiwaRobot.IT5 IiwaRobot.IT6 IiwaRobot.IT7; IiwaRobot.mass];
    end
    properties (Constant, Access='private')
        CM1 = [0; -0.03; 0.2775]; 
        CM2 = [0; 0.042; 0.419]; 
        CM3 = [0; 0.03; 0.6945];
        CM4 = [0; -0.034; 0.847]; 
        CM5 = [0; -0.021; 1];
        CM6 = [0; 0.001; 1.18]; 
        CM7 = [0; 0; 1.28];
        IT1 = [0.1; 0.09; 0.02]; 
        IT2 = [0.018; 0.05; 0.044];
        IT3 = [0.08; 0.075; 0.01];
        IT4 = [0.03; 0.029; 0.01]; 
        IT5 = [0.02; 0.018; 0.005];
        IT6 = [0.005; 0.0036; 0.0047]; 
        IT7 = [0.001; 0.001; 0.001];
        mass = [4 4 3 2.7 1.7 1.8 0.3];
    end
    
    methods
        function obj = IiwaRobot()
        end
    end
end

