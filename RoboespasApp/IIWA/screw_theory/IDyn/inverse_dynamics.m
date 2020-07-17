function [x_forcetorque] = inverse_dynamics(q_curr, q_des)%, qdot, qtorque)
    PoAcc = [0 0 -9.80665]';
    LiMas = [0         0         0         0         0         0         0;
       -0.0300    0.0420    0.0300   -0.0340   -0.0210    0.0004         0;
        0.2775    0.4190    0.6945    0.8470    1.0410    1.1810    1.2810;
        0.1000    0.0180    0.0800    0.0300    0.0200    0.0050    0.0010;
        0.0900    0.0500    0.0750    0.0290    0.0180    0.0036    0.0010;
        0.0200    0.0440    0.0100    0.0100    0.0050    0.0047    0.0010;
        4.0000    4.0000    3.0000    2.7000    1.7000    1.8000    0.3000];
    [~, ~, ~, Twist, ~, ~, ThDotMax] = data_IIWA();
    %Tdynmax = [320 320 176 176 110 40 40];
    n=7;
    ttall = max(2*(ThDotMax.^-1)*diag(abs(q_des-q_curr)));
    if ttall == 0
        qdot = zeros(1,n);
        qdotdot = zeros(1,n);
    else
        qdot = 2*(q_des-q_curr)/ttall;
        qdotdot = qdot/ttall;   
    end
    
    TwMag = [Twist; q_des];
    
    MInertia = MInertiaJsl(TwMag,LiMas);
    CCoriolis = CCoriolisAij(TwMag,LiMas,qdot);
    NPotential = NPotentialWre(TwMag,LiMas,PoAcc);

    x_forcetorque = MInertia*qdotdot' + CCoriolis*qdot' + NPotential;  
end

