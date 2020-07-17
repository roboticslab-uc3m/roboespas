%FUNCIONA: Programa completo: lee un modelo y un trc, arregla el trc, ejecuta
%cinemática inversa y lo muestra en pantalla

clear all
close all
clc
import org.opensim.modeling.*

%% INPUTS

cd('D:\User\escrit\Universidad\Master\2\TFM\MODELO')
model = Model('MoBL_ARMS_module2_4_allmuscles_1Marker.osim');
trcfile='Lab2.trc';

%% Main

% TRCFileFixer(trcfile)

tool=InverseKinematicsTool()
tool.setModel(model);
tool.setStartTime(0.4);
tool.setEndTime(1.6);
tool.setMarkerDataFileName('Lab2.trc')
tool.setOutputMotionFileName('output.mot')
tool.run();

%% Visualizar en pantalla
MotName='output.mot'
RepEnPantalla(model,MotName)