classdef IiwaParameters
    %% IiwaParameters
    % Clase que contiene todos los parametros del IIWA
    % * PoAcc: P
    properties (Constant, Access = 'public')
        PoAcc = [0 0 -9.81]';
        l_tool = 0.152;
        n_joints = 7;
        pk=[0;0;0.36]; 
        pf=[0;0;1.18]; 
        pp=[0;0; 1.306 + IiwaParameters.l_tool];
        Twist = [0   -0.3600         0    0.7800         0   -1.1800         0;
                 0         0         0         0         0         0         0;
                 0         0         0         0         0         0         0;
                 0         0         0         0         0         0         0;
                 0    1.0000         0   -1.0000         0    1.0000         0;
            1.0000         0    1.0000         0    1.0000         0     1.000];
        Hst0 = [1 0 0 0; 0 1 0 0; 0 0 1 1.306 + IiwaParameters.l_tool; 0 0 0 1];
        % Maximum Magnitude for the robot joints POSITION rad, (by catalog).
        Thmax = deg2rad([170 120 170 120 170 120 175]);
        ThDotmax = deg2rad([85, 85, 100, 75, 130, 135, 135]);%75 75 90 90 144 135 135]);
        Torquemax = [176 176 110 110 110 40 40]; %Nm
        CartAccMax = 0.5 %m/s2
        LiMas = [IiwaParameters.CM1 IiwaParameters.CM2 IiwaParameters.CM3 IiwaParameters.CM4 ...
            IiwaParameters.CM5 IiwaParameters.CM6 IiwaParameters.CM7;
            IiwaParameters.IT1 IiwaParameters.IT2 IiwaParameters.IT3 IiwaParameters.IT4 ...
            IiwaParameters.IT5 IiwaParameters.IT6 IiwaParameters.IT7; IiwaParameters.mass];
    end
    properties (Constant, Access='private')
        CM1 = [0; -0.03; 0.2775];  %[-0.1, 0, 0.7] %[0 -0.03 0.12]
        CM2 = [0; 0.042; 0.419]; %[0, 0.059, 0.042]
        CM3 = [0; 0.03; 0.6945]; % 
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
        function obj = IiwaParameters()
        end
    end
end

