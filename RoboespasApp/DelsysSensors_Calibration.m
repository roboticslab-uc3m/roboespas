%% Calibración de los sensores inerciales de Delsys
% Esta calibración de IMUs se basa en el procedimiento propuesto en 
%    Tedaldi, D., Pretto, A., & Menegatti, E. (2014, May). 
%    A robust and easy to implement method for IMU calibration without external equipments. 
%    In 2014 IEEE International Conference on Robotics and Automation (ICRA) (pp. 3042-3049). IEEE.
% La calibración de los sensores inerciales se compone de dos partes
%  1. Toma de datos estáticos para determinar el tiempo mínimo estático
%  inicial necesario para una correcta calibración de los sensores
%  2. Toma de datos de cada sensor en múltiples orientaciones para llevar a
%  cabo la calibración. Mantener los sensores horizontales y estáticos el tiempo inicial determinado en
%  el paso anterior antes de proceder a colocarlos en múltiples
%  orientaciones, dejándolos 5s estáticos en cada orientación (mínimo 35 orientaciones diferentes)

%% Toma de datos
% delsys = DelsysSensors(1);
% delsys.SetMovement('calib');
% delsys.SetSensorsPosition(zeros(6,3));
% delsys.StartSensors();
% delsys.StartCapture();
% delsys.EndReference(); % Para la parte 1, determinación del tiempo estático inicial
% delsys.EndTrial(1); % Para la parte 2, correspondiéndose el número del trial con el sensor que se calibra
% delsys.StopSensors();
% delsys.CloseSensors();

%% Parte 1: Determinación del tiempo inicial estático mínimo Tinit
% NumSensors = 6;
% 
% for i = 1:NumSensors
%     str = sprintf('Sensor%d', i);
%     gyro{i} = delsys.DelsysReference.(str).IMU.Gyro';
% end
% t_total = delsys.DelsysReference.Sensor1.IMU.Timestamps;
% 
% % Calcular Tinit mediante la varianza de Allan
% t_intervals = 1:floor(t_total(end));
% fs = 1/(t_total(2)-t_total(1));
% for i = 1:NumSensors
%     [avar, tau] = allanvar(gyro{i}, t_intervals, fs);
%     figure(i);
%     xlim([0 240])
%     plot(t_intervals, avar);
% end

%% Parte 2: Calibración de los sensores
% Declaración de variables
Tinit = 100; % Obtenido visualmente de las gráficas de la parte 1
error = 5e-3;
numSensors = max(size(delsys.DelsysTrial));
window = 153; % 1s aprox

t_att = cell(numSensors,1);

acc_att = cell(numSensors,1);
acc_calibrated = cell(numSensors,1);
thetaAcc_opt = cell(numSensors,1);

gyro_att = cell(numSensors,1);
gyro_calibrated = cell(numSensors,1);
thetaGyro_opt = cell(numSensors,1);

var_init = zeros(1,numSensors);

for sensor = 1:numSensors
    str = sprintf('Sensor%d', sensor);
    
    % Importar datos del sensor
    t_att{sensor} = delsys.DelsysTrial{sensor}.(str).IMU.Timestamps';
    tinit_sample = find(abs(t_att{sensor}-Tinit)<=error, 1);
    fs = 1/(t_att{sensor}(2) - t_att{sensor}(1));
    
    acc_att{sensor} = delsys.DelsysTrial{sensor}.(str).IMU.Acc; 

    gyro_att{sensor} = delsys.DelsysTrial{sensor}.(str).IMU.Gyro;

    % Calcular la varianza del sensor durante Tinit (período estático inicial)
    var_init(sensor) = (var(acc_att{sensor}(1,1:tinit_sample)))^2 + (var(acc_att{sensor}(2,1:tinit_sample)))^2 + (var(acc_att{sensor}(3,1:tinit_sample)))^2;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% CALIBRACIÓN DEL ACELERÓMETRO %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    theta0_Acc = [0 0 0 0 0 0 0 0 0]; % theta0_Acc = [alphaYZ, alphaZY, alphaZX, Sa_x, Sa_y, Sa_z, bx, by, bz]
    max_iters = 30;
    Minf = zeros(9+1+1, max_iters); % [theta_Acc'; resnorm; threshold]

    VAR = movvar(acc_att{sensor}(1,:), window).^2 + movvar(acc_att{sensor}(2,:), window).^2 + movvar(acc_att{sensor}(3,:), window).^2;

    for iter = 1:max_iters
        threshold = iter*var_init(sensor);
        
        % Detección de intervalos estáticos
        static_idx = VAR<threshold; 
        s_intervals = [(acc_att{sensor}(1,:).*static_idx); (acc_att{sensor}(2,:).*static_idx); (acc_att{sensor}(3,:).*static_idx)];
        [s_intervals_noNoise, qs] = staticIntervals(s_intervals, window);

        % Estimación de los parámetros de calibración theta_Acc
        options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'Display', 'none','MaxFunEvals', 150000, 'MaxIter', 6000, 'TolFun', 10^(-10));
        Lacc_handle = @(theta0)Lacc(theta0, s_intervals_noNoise); 
        [theta_Acc,resnorm, ~,~,~,~,~] = lsqnonlin(Lacc_handle, theta0_Acc, [], [], options); 
        Minf(:,iter) = [theta_Acc'; resnorm; threshold];
    end

    % Establecer valores óptimos de calibración
    [~, indexOpt] = min(Minf(10,:));
    threshold_opt = Minf(11,indexOpt);
    thetaAcc_opt{sensor} = Minf(1:9, indexOpt);
    static_idx_opt = VAR<threshold_opt;

    % Calibrar acelerómetro
    acc_calibrated{sensor} = calibrateAcceleration(thetaAcc_opt{sensor}, acc_att{sensor});

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% CALIBRACIÓN DEL GIRÓSCOPO %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Determinar primer período estático largo para calcular el offset del sensor
    start_idx = find(static_idx_opt == 1, 1);
    end_idx = find(static_idx_opt(start_idx:end)==0,1) + start_idx - 2;
    offset_gyro(:,sensor) = mean(gyro_att{sensor}(:,start_idx:end_idx),2);
    gyroNO = gyro_att{sensor} - diag(offset_gyro(:,sensor))*ones(3,length(gyro_att{sensor}(1,:)));

    % Detección de intervalos estáticos óptimos
    s_intervals_calibrated = [(acc_calibrated{sensor}(1,:).*static_idx_opt); (acc_calibrated{sensor}(2,:).*static_idx_opt); (acc_calibrated{sensor}(3,:).*static_idx_opt)];
    [s_intervals_calibratedNO, qs_calibrated] = staticIntervals(s_intervals_calibrated, window);

    % Estimación de los parámetros de calibración thetaGyro
    % n = 1;% number of bits of the A/D converter -> no tengo datos del A/D converter...
    % y = deg2rad(2000); % gyroscope full range [-y,+y] in rad/s -> range [+- 2000 dps] 
    % r = (2^n - 1)/(2*y);
    % % theta0_Gyro = [1/r 0 0 0 1/r 0 0 0 1/r]; 
    theta0_Gyro = [1e-04 0 0 0 1e-04 0 0 0 1e-04]; % theta0_Gyro = [gammaYZ, gammaZY, gammaXZ, gammaZX, gammaXY, gammaYX, Sg_x, Sg_y, Sg_z]
    options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'Display', 'none','TolX', 10^-7, 'TolFun' , 10^-6, 'MaxFunEvals', 400);
    Lgyro_handle = @(theta0_Gyro) Lgyro(theta0_Gyro, qs_calibrated, gyroNO);
    thetaGyro_opt{sensor} = lsqnonlin(Lgyro_handle, theta0_Gyro, [], [], options);
    
    % Calibrar giróscopo
    gyro_calibrated{sensor} = calibrateAngVel(thetaGyro_opt{sensor}, gyroNO);
end

%% Funciones
function [q, angularRotation] = angVel2Q( angVel, dt )
    s = size(angVel,2);
    angVel_norm = zeros(1,s);
    angularRotation = zeros(1,s);
    direction = zeros(3,s);
    q = zeros(s,4);

    for i = 1:s
        angVel_norm(i) = (angVel(1,i)^2 + angVel(2,i)^2 + angVel(3,i)^2)^(1/2);
        angularRotation(i) = angVel_norm(i)*dt;
        direction(:,i) = angVel(:,i)/angVel_norm(i);
        q(i, :) = [cos(angularRotation(i)/2), sin(angularRotation(i)/2)*(direction(:,i)')];
    end
end

function [acc_calibrated] = calibrateAcceleration(theta, acc)
    T_a =  [1   -theta(1)    theta(2)
            0       1       -theta(3)
            0       0              1];
    K_a = [theta(4)      0          0
            0          theta(5)     0
            0            0        theta(6)];
    b_a = [theta(7), theta(8), theta(9)]';
    acc_calibrated = T_a*K_a*acc - diag(b_a)*ones(3, length(acc)); 
%     acc_calibrated = (T_a*K_a*(acc + b_a));
end

function [gyro_calibrated] = calibrateAngVel(theta, gyro)
    Tgyro = [   1       -theta(1)     theta(2)
              theta(3)       1       -theta(4) 
             -theta(5)   theta(6)           1];
    Kgyro = diag([theta(7), theta(8), theta(9)]);
    
    gyro_calibrated = Tgyro*Kgyro*gyro;
end

function [s_intervals_noNoise,qs] = staticIntervals(s_intervals, window)
    ne0 = find(s_intervals(3,:)~=0);                   % Nonzero Elements (transposed)
    ix0 = unique([ne0(1) ne0(diff([0 ne0])>1)]);        % Segment Start Indices
    ix1 = ne0([find(diff([ne0(1) ne0])>1)-1 length(ne0)]);   % Segment End Indices
    for i = 1:length(ix0)-1
        dyn = ix0(i+1) - ix1(i) - 1;
        if dyn <= 30
            s_intervals(:,ix1(i)+1:ix1(i)+dyn) = diag(s_intervals(:,ix1(i)))*ones(3,dyn);
        end
    end
    ne0 = find(s_intervals(3,:)~=0);                   % Nonzero Elements (transposed)
    ix0 = unique([ne0(1) ne0(diff([0 ne0])>1)]);        % Segment Start Indices
    ix1 = ne0([find(diff([ne0(1) ne0])>1)-1 length(ne0)]);   % Segment End Indices
    s_intervals_cell = cell(1,length(ix0));
    for k1 = 1:length(ix0)
        s_intervals_cell{k1} = movmean(s_intervals(:,ix0(k1):ix1(k1)),window,2);
    end
    sizes = cellfun('size', s_intervals_cell, 2);
    wrongInterval = find(sizes<=window);
    if ~isempty(wrongInterval)
        sizes(wrongInterval) = [];
        ix0(wrongInterval) = [];
        ix1(wrongInterval) = [];
        s_intervals_cell(wrongInterval) = [];
    end
    minsize = min(sizes);
    s_intervals_cell = cellfun(@(x) x(:,1:minsize),s_intervals_cell,'UniformOutput',false);
    
    s_mean = cellfun(@(x) mean(x,2), s_intervals_cell, 'UniformOutput', 0);
    s_intervals_mean = reshape(cell2mat(s_mean), 3, length(s_mean));
    qs = [ix0; ix1; sizes; s_intervals_mean];
    s_intervals_noNoise = cell2mat(s_intervals_cell);
end

function [rotMatrix] = RK4n(w)
    % delta_t = sampling time
    dt = 0.0068;
    
    wx = w(1,:);
    wy = w(2,:);
    wz = w(3,:);
    
    % Initialize q_k and q_k_next
    q_k = angVel2Q(w(:,2), dt)';
    q_k_next = [0; 0; 0; 0];
    
    for i = 2:size(wx,2)-2
        % 1st RK coefficient - k1, c1 = 0
        q1 = q_k;
        omega_wt0 = [0       -wx(i)    -wy(i)    -wz(i)
                     wx(i)     0        wz(i)    -wy(i)
                     wy(i)   -wz(i)      0        wx(i)
                     wz(i)    wy(i)    -wx(i)        0];
        k1 = (1/2)*omega_wt0*q1;
        
        % 2nd RK coefficient - k2, c2 = 0.5, a21 = 0.5
        q2 = q_k + dt*0.5*k1;
        omega_wt05 = [0                    -(wx(i)+wx(i+1))/2    -(wy(i)+wy(i+1))/2     -(wz(i)+wz(i+1))/2
                       (wx(i)+wx(i+1))/2            0              (wz(i)+wz(i+1))/2     -(wy(i)+wy(i+1))/2
                       (wy(i)+wy(i+1))/2    -(wz(i)+wz(i+1))/2             0              (wx(i)+wx(i+1))/2
                       (wz(i)+wz(i+1))/2     (wy(i)+wy(i+1))/2    -(wx(i)+wx(i+1))/2                      0];
        k2 = (1/2)*omega_wt05*q2;
        
        % 3rd RK coefficient - k3, c3 = 0.5, a31 = 0, a32 = 0.5, same omega than k2
        q3 = q_k + dt*0.5*k2;
        k3 = (1/2)*omega_wt05*q3;
        
        % 4th RK coefficient - k4, c4 = 1, a41 = 0, a42 = 0, a43 = 1
        q4 = q_k + dt*k3;
        omega_wt1 = [0         -wx(i+1)    -wy(i+1)    -wz(i+1)
                     wx(i+1)       0        wz(i+1)    -wy(i+1)
                     wy(i+1)   -wz(i+1)        0        wx(i+1)
                     wz(i+1)    wy(i+1)    -wx(i+1)          0];
        k4 = (1/2)*omega_wt1*q4;
        
        q_k_next = q_k + dt*(1/6)*(k1 + k2 + k3 + k4);
        q_k_next = q_k_next/norm(q_k_next);
        
        q_k = q_k_next;
    end
    rotMatrix = rotmat(quaternion(q_k_next'),'frame');
end

function [res_vector] = Lgyro(theta, qs, gyro)
    % theta0_Gyro = [gammaYZ, gammaZY, gammaXZ, gammaZX, gammaXY, gammaYX, Sg_x, Sg_y, Sg_z]
    Tgyro = [   1       -theta(1)   theta(2)
             theta(3)       1       -theta(4) 
             -theta(5)  theta(6)        1];
    Kgyro = diag([theta(7), theta(8), theta(9)]);
    
    h_gyro = Tgyro*Kgyro*gyro;
    
    residuals = zeros(size(qs,2)-1, 1);
    
    for interval = 1:size(qs,2)-1
        d_intervals = h_gyro(:,(qs(2,interval)+1):(qs(1, interval+1)-1));
        rotMatrix = RK4n(d_intervals);
        norm_i = (qs(4,interval)^2 + qs(5,interval)^2 + qs(6,interval)^2)^(1/2);
        norm_i_next = (qs(4,interval+1)^2 + qs(5,interval+1)^2 + qs(6,interval+1)^2)^(1/2);
        v = (qs(4:6,interval+1)/norm_i_next - rotMatrix*qs(4:6,interval)/norm_i)';
        residuals(interval, 1) = (v(1)^2 + v(2)^2 + v(3)^2)^(1/2);
    end
    
    res_vector = residuals;
end

function [res_vector] = Lacc(theta, acc)
    residuals = zeros(length(acc(1,:)),1);
    g = 9.79956;
    % theta = [alphaYZ, alphaZY, alphaZX, Sx, Sy, Sz, bx, by, bz]
    Tacc = [1   -theta(1)    theta(2)
            0       1       -theta(3)
            0       0          1    ];
    Kacc = [theta(4)    0           0
            0           theta(5)    0
            0           0           theta(6)];
    b_a = [theta(7), theta(8), theta(9)]';
    
%     h_acc = Tacc*Kacc*(acc + b_a);
    h_acc = Tacc*Kacc*acc - diag(b_a)*ones(3, length(acc));  % Esto según el código de los del artículo
    for k = 1:length(acc(1,:))
        residuals(k,1) = g^2 - (h_acc(1,k)^2 + h_acc(2,k)^2 + h_acc(3,k)^2);
    end
    res_vector = residuals;
end