% A PARTIR DE UN MUSCLE, CREO UN MILLARD Y LO REPRESENTO
clear all
close all
clc
import org.opensim.modeling.*
myModel = Model('Arm_Flexion_Millard.osim');
model_muscles = myModel.getMuscles();
BRA = model_muscles.get('BRA');
    MIF = BRA.getMaxIsometricForce()
    OFL=BRA.getOptimalFiberLength()
    TSL=BRA.get_tendon_slack_length()
    PA=BRA.getPennationAngleAtOptimalFiberLength()

MillardMuscle = org.opensim.modeling.Millard2012EquilibriumMuscle('BRAm',MIF,OFL,TSL,PA);
ffl=MillardMuscle.getFiberForceLengthCurve();
afl=MillardMuscle.getActiveForceLengthCurve();
fv=MillardMuscle.getForceVelocityCurve();
tfl=MillardMuscle.getTendonForceLengthCurve();

ffl.printMuscleCurveToCSVFile('D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M\Curves');
afl.printMuscleCurveToCSVFile('D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M\Curves');
fv.printMuscleCurveToCSVFile('D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M\Curves');
tfl.printMuscleCurveToCSVFile('D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M\Curves');

afl_data=importdata("D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M\Curves\ActiveForceLengthCurve.csv");
ffl_data=importdata("D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M\Curves\FiberForceLengthCurve.csv");
fv_data=importdata("D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M\Curves\ForceVelocityCurve.csv");
tfl_data=importdata("D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M\Curves\TendonForceLengthCurve.csv");

figure(1)
subplot(2,2,1)
plot(afl_data.data(:,1),afl_data.data(:,2))
title('Active Fiber Force Length')
hold on
subplot(2,2,2)
plot(ffl_data.data(:,1),ffl_data.data(:,2))
title('Passive Fiber Force Length')
hold on
subplot(2,2,3)
plot(fv_data.data(:,1),fv_data.data(:,2))
title('Force Velocity')
hold on
subplot(2,2,4)
plot(tfl_data.data(:,1),tfl_data.data(:,2))
title('Tendon Force Length')

figure(2) %TOTAL FIBER FORCE LENGTH
plot(afl_data.data(:,1),afl_data.data(:,2))
hold on
plot(ffl_data.data(:,1),ffl_data.data(:,2))
title('TOTAL FiberForceLength')


%Sumar ambas curvas: NO FUNCIONA
% Graph_1 = afl_data.data(:,1:2)
% Mtx_1 = reshape(Graph_1, 2, [])';
% Graph_2 = ffl_data.data(:,1:2)
% Mtx_2 = reshape(Graph_2, 2, [])';
% xmax1 = max(Mtx_1(:,2));                                        % Find Maximum ‘x’ Of Shorter Matrix
% Rows2 = (Mtx_2(:,2) <= xmax1);                                  % Limit Longer Matrix To That Value
% Mtx_C = [Mtx_1; Mtx_2(Rows2,:)];                                % Create New ‘Concatenated’ Matrix
% xv = linspace(min(Mtx_C(:,2)), max(Mtx_C(:,2)));                % X-Vector For Plot
% b = polyfit(Mtx_C(:,2), Mtx_C(:,1), 5);                         % Fit Data
% pv = polyval(b, xv);                                            % Evaluate Polynomial Fit
% figure(3)
% plot(Mtx_1(:,2), Mtx_1(:,1))
% hold on
% plot(Mtx_2(:,2), Mtx_2(:,1))
% plot(xv, pv, '-g', 'LineWidth',2)
% hold off
% grid


