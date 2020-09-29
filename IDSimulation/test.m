%% Screw Theory - CLASSICAL INVERSE DYNAMICS - KUKA IIWA14.
% M(t)*ddt + C(t,dt)*dt + N(t,dt) = T
clear
clc
%
Twist = IiwaRobot.Twist;
LiMas = IiwaRobot.LiMas;
% Potential Action Vector - Gravity definition (i.e., -g direction).
PoAcc = [0 0 -9.81]';

Th = [170*(rand-rand) 120*(rand-rand) 170*(rand-rand) 120*(rand-rand)];
Th = [Th 170*(rand-rand) 120*(rand-rand) 175*(rand-rand)];
Th = Th*pi/180;
Thp = [85*(rand-rand) 85*(rand-rand) 100*(rand-rand)];
Thp = [Thp 75*(rand-rand) 130*(rand-rand) 135*(rand-rand) 135*(rand-rand)];
Thp = Thp*pi/180;
Thpp = [(rand-rand)*Thp(1) (rand-rand)*Thp(2) (rand-rand)*Thp(3)];
Thpp = [Thpp (rand-rand)*Thp(4) (rand-rand)*Thp(5) (rand-rand)*Thp(6)];
Thpp = [Thpp (rand-rand)*Thp(7)];

tic; Tdyn = IiwaScrewTheory.InverseDynamics(Th, Thp, Thpp); toc;
