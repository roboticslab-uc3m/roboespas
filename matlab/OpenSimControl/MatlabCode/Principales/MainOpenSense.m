% Este programa lee los vectores de posici�n y fuerzas de la herramienta del robot en
% un archivo .mat y los archivos proporcionados por la Kinect en un archivo CSV.
% Transforma estas tablas en una tabla .trc para que pueda
% ser utilizada por OpenSim y realizar una cinem�tica inversa. Una vez
% obtenido el archivo .mot, mediante IK, con el registro de los movimientos de cada
% coordenada del modelo, se puede lanzar un visualizador para que
% represente estos movimientos. El programa tras calcular la IK, calcula las fuerzas externas
% que se aplican sobre el brazo del paciente, para, a continuaci�n, realizar un an�lisis CMC,
% que devolver� los valores de excitationes y activaciones. Tras este c�lculo, se ejecuta una
% �ltima herramienta (Forward Dynamics) que calcula el resto de valores necesarios para la investigaci�n.
% Adicionalmente el programa ejecuta una
% dinamica inversa a partir del archivo .mot, a la que se le podr�a sumar
% unas fuerzas externas que interviniesen en el proceso (las realizadas por
% el brazo robotico).

clear all
close all
clc
import org.opensim.modeling.*
format long
pause(2);
%% INPUTS

%   <<<<<<<<<<<<<MODIFICABLES:>>>>>>>>>>>>
% Masa del sujeto
masaTotal=67;
% mass=4.77882; %4.77882 es la que tiene el modelo por defecto
mass = masaTotal*0.55; % la masa del tronco superior es aproximadamente el 55% de la masa  total

escalarModelo=false;
methodIIWA_FD= 'Screw Theory'; %'Matlab toolbox'   Screw Theory
modeladoMuscular='Sano_Millard'; %'Spastic_Millard'   Sano_Thelen   Spastic_Thelen   Sano_Millard
grado=4; % Regresi�n fuerzas Screw Theory
version='V2'; % V2 %'V1' para usar el modelo de Eduardo, 'V2' para usar el modelo de Anaelle

file_path = which(mfilename);
id_ch_folders = find(file_path =='\');
pathOpenSimControl = file_path(1:id_ch_folders(end-2));
disp(['Using OpenSimControl path: ', pathOpenSimControl]);

pathOpenSim = ['C:\OpenSim ', char(org.opensim.modeling.opensimCommon.GetVersion())];
% title=strcat('Selecciona el directorio de instalaci�n de OpenSim') ;
% pathOpenSim = uigetdir(pathOpenSim, title);
% if (pathOpenSim == 0)
%     ME = MException('Main:NoOpenSimPath', 'No OpenSim path selected');
%     throw(ME);
% end
% disp(['Using OpenSim installation path: ', pathOpenSim]);

% Path model folder
CD_model= [pathOpenSimControl, 'ROBOESPAS_FLEXION'];

%% Load data
% Get model name, load plugin in OpenSim if its spastic
switch modeladoMuscular
    case 'Spastic_Thelen'
        opensimCommon.LoadOpenSimLibrary("..\Plugins\SpasticThelenMuscleModel.dll")
        MODELO = ['Arm_Flexion_SpasticThelen_', version, '.osim'];
    case 'Spastic_Millard'
        opensimCommon.LoadOpenSimLibrary("..\Plugins\SpasticMillardMuscleModel.dll")
        MODELO = ['Arm_Flexion_SpasticMillard_', version, '.osim'];
    case 'Sano_Thelen'
        MODELO = ['Arm_Flexion_Thelen_', version, '.osim'];
    case 'Sano_Millard'
        MODELO = ['Arm_Flexion_Millard_', version, '.osim'];
end

% Load trial
IMUpath = strcat(CD_model, '\IMUData');
load(strcat(IMUpath, '\trialOpenSense.mat'));

%% Create Xsens file from trial
f_createXsensFile(trial, IMUpath);

%% Convert IMU data to OpenSim format
% Build an Xsens Settings Object. 
% Instantiate the Reader Settings Class
xsensSettings = XsensDataReaderSettings(strcat(IMUpath, '\myIMUMappings2.xml'));
% Instantiate an XsensDataReader
xsens = XsensDataReader(xsensSettings);
% Get a table reference for the data
tables = xsens.read(strcat(IMUpath, '/'));
% get the trial name from the settings
trial = char(xsensSettings.get_trial_prefix());

% Get Orientation Data as quaternions
quatTable = xsens.getOrientationsTable(tables);
% Write to file
STOFileAdapterQuaternion.write(quatTable,  [IMUpath, '\', trial '_orientations.sto']);

%% Calibrate OpenSim model
% Set variables to use
MODELO = [CD_model, '\', MODELO];
orientationsFileName = [IMUpath,'/PruebaFlexExt_orientations.sto'];%'MT_012005D6_009-001_orientations.sto';   % The path to orientation data for calibration 
sensor_to_opensim_rotation = Vec3(0, pi/2, 0);% The rotation of IMU data to the OpenSim world frame 
% baseIMUName = 'humerus_imu';                     % The base IMU is the IMU on the base body of the model that dictates the heading (forward) direction of the model.
% baseIMUHeading = 'y';                           % The Coordinate Axis of the base IMU that points in the heading direction. 
visulizeCalibration = false;                     % Boolean to Visualize the Output model

% Instantiate an IMUPlacer object
imuPlacer = IMUPlacer();

% Set properties for the IMUPlacer
imuPlacer.set_model_file(MODELO);
imuPlacer.set_orientation_file_for_calibration(orientationsFileName);
imuPlacer.set_sensor_to_opensim_rotations(sensor_to_opensim_rotation);
% imuPlacer.set_base_imu_label(baseIMUName);
% imuPlacer.set_base_heading_axis(baseIMUHeading);

% Run the IMUPlacer
imuPlacer.run(visulizeCalibration);

% Get the model with the calibrated IMU's
model = imuPlacer.getCalibratedModel();

% Print the calibrated model to file.
model.print(strrep(MODELO, '.osim', '_calibrated.osim') );

%% Obtain IK motion from orientation tracking
% Set variables to use
MODELO = [CD_model, '\Arm_Flexion_Millard_V2_calibrated.osim']; %'Rajagopal_2015_calibrated.osim';                % The path to an input model
visualizeTracking = false;  % Boolean to Visualize the tracking simulation
startTime = 22; %7.25;          % Start time (in seconds) of the tracking simulation. 
endTime = 27; %15;              % End time (in seconds) of the tracking simulation.
resultsDirectory = [CD_model,'\IKResults'];

% Instantiate an InverseKinematicsTool
imuIK = IMUInverseKinematicsTool();
 
% Set the model path to be used for tracking
imuIK.set_model_file(MODELO);
imuIK.set_orientations_file(orientationsFileName);
imuIK.set_sensor_to_opensim_rotations(sensor_to_opensim_rotation)
% Set time range in seconds
imuIK.set_time_range(0, startTime); 
imuIK.set_time_range(1, endTime);   
% Set a directory for the results to be written to
imuIK.set_results_directory(resultsDirectory)
% Run IK
imuIK.run(visualizeTracking);

OutputMotionStr = 'ik_PruebaFlexExt_orientations.mot';
motFilePath=strcat(CD_model,'\IKResults\',OutputMotionStr);
%% CMC
import org.opensim.modeling.*;
CD_cmc=strcat(CD_model,'\CMC');
%Coordenadas (todas)
CoordSet= model.getCoordinateSet();
elv_angle=CoordSet.get('elv_angle');
shoulder_elv=CoordSet.get('shoulder_elv');
shoulder_rot=CoordSet.get('shoulder_rot');
elbow_flexion=CoordSet.get('elbow_flexion');
pro_sup=CoordSet.get('pro_sup');
deviation=CoordSet.get('deviation');
flexion=CoordSet.get('flexion');
r_z = CoordSet.get('r_z');
%Bloqueos:
elv_angle.set_locked(0);
shoulder_elv.set_locked(0);
shoulder_rot.set_locked(0);
elbow_flexion.set_locked(0);
pro_sup.set_locked(1);
deviation.set_locked(1);
flexion.set_locked(1);
r_z.set_locked(1);
model.print(MODELO);

sto=org.opensim.modeling.Storage(motFilePath);
% Tiempos:
StartTime=sto.getFirstTime;
LastTime=sto.getLastTime;


%     cmcTool=CMCTool(strcat(CD_cmc,"\CMC_Setup_Roboespas_Flex.xml"));
cmcTool=CMCTool();
% Name
cmcTool.setName('CMCtool');
% Model
cmcTool.setModel(model);
cmcTool.setModelFilename(strcat(CD_model,'\',MODELO));
% Replace Force Set
cmcTool.setReplaceForceSet(false);
% Results
cmcTool.setResultsDir(strcat(CD_model,'\CMCResults'));
% Output precision
cmcTool.setOutputPrecision(20);
% Time Range
cmcTool.setStartTime(StartTime);
cmcTool.setFinalTime(LastTime);
% Solve For Equilibrium
cmcTool.setSolveForEquilibrium(true);
% Maximum number of Integrator Steps
cmcTool.setMaximumNumberOfSteps(1);
% Maximum integration step size
cmcTool.setMaxDT(1e-03);
% Minimum integration step size
cmcTool.setMinDT(1e-03);
% Integrator error tolerance
cmcTool.setErrorTolerance(1e-04);
% External Loads File
% cmcTool.setExternalLoadsFileName(xmlExternalLoadsFileName_FT);
% Filtered Motion
cmcTool.setDesiredKinematicsFileName(motFilePath);
% Tasks
cmcTool.setTaskSetFileName(strcat(CD_cmc,'\CMC_Tasks-ROBOESPAS_flex.xml'));
% Actuator Constraints
cmcTool.setConstraintsFileName(strcat(CD_cmc,'\CMC_ControlConstraints_ROBOESPAS.xml'));
%  Low Pass Frequency
cmcTool.setLowpassCutoffFrequency(6);
% Look ahead window time
cmcTool.setTimeWindow(0.01);

% Fast Optimization Target. True -> mas rapido y preciso pero tiene que
% cumplir las aceleraciones. En el GUI esta puesto por defecto.
% POR EL MOMENTO NO SE PUEDE UTILIZAR DEBIDO A QUE EL MODELO OSIM UTILIZADO
% NO TIENE BIEN DEFINIDAS LAS DIN�MICAS. SI EN UN FUTURO SE DISPONE DE OTRO
% MODELO QUE CUMPLA ESTO, O SE CONSIGUEN MEJORAR ESTAS DIN�MICAS, SE DEBER�
% USAR LA OPCI�N FastTarget
cmcTool.setUseFastTarget(false);

% % Points: No es necesario
% %   cmcTool.setDesiredPointsFileName(motFilePath);

cmcTool.print(strcat(CD_cmc,'\setupActualCMC.xml'));

cmc = CMCTool(strcat(CD_cmc,'\setupActualCMC.xml'));
% cmc.run; %Esto se hace por seguridad. Recomendaci�n de OpenSim.
% cmcTool.run;

disp('CMC Completado');

%% Representar excitaciones estimadas con CMC
% [cmcExcitations] = f_importCMCExcitations(strcat(CD_cmc, 'Results\CMCtool_controls.sto'));
% muscles = fieldnames(cmcExcitations);
% numMuscles = length(muscles)-1;
% f = figure('Name', 'Estimated EMG', 'WindowState', 'maximized');
% for i = 1:numMuscles
%     subplot(numMuscles,1,i, 'Parent',f);
%     plot(cmcExcitations.time, cmcExcitations.(muscles{i+1}))
%     title(muscles{i+1})
%     drawnow
% end