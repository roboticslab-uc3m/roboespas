function [pk, pf, pp, Twist, Hst0, Thmax, ThDotmax] = data_IIWA()
%     % Function that sets the IIWA parameters
%     AxisX = [1 0 0]'; AxisY = [0 1 0]'; AxisZ = [0 0 1]';
%     po=[0;0;0]; pk=[0;0;0.36]; pr=[0;0;0.78]; pf=[0;0;1.18]; pp=[0;0;1.18];
%     Point = [po pk po pr po pf po];
%     Joint = ['rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'; 'rot'];
%     Axis = [AxisZ AxisY AxisZ -AxisY AxisZ AxisY AxisZ];
%     Twist = zeros(6,n);
%     for i = 1:n
%         Twist(:,i) = joint2twist(Axis(:,i), Point(:,i), Joint(i,:));
%     end

    l_tool=0.152; %ManetaFT
    pk=[0;0;0.36]; pf=[0;0;1.18]; pp=[0;0;1.306+l_tool];
    Twist = [0   -0.3600         0    0.7800         0   -1.1800         0;
             0         0         0         0         0         0         0;
             0         0         0         0         0         0         0;
             0         0         0         0         0         0         0;
             0    1.0000         0   -1.0000         0    1.0000         0;
        1.0000         0    1.0000         0    1.0000         0     1.000];
    Hst0 = [1 0 0 0; 0 1 0 0; 0 0 1 1.306+l_tool; 0 0 0 1];
    % Maximum Magnitude for the robot joints POSITION rad, (by catalog).
    Thmax = deg2rad([170 120 170 120 170 120 175]);
    ThDotmax = deg2rad([67 67 81 81 130 120 120]);
    
end