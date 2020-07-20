% Este programa lee los vectores de posiciï¿½n y fuerzas de la herramienta del robot en
% un archivo .mat y los archivos proporcionados por la Kinect en un archivo CSV.
% Transforma estas tablas en una tabla .trc para que pueda
% ser utilizada por OpenSim y realizar una cinemï¿½tica inversa. Una vez
% obtenido el archivo .mot, mediante IK, con el registro de los movimientos de cada
% coordenada del modelo, se puede lanzar un visualizador para que
% represente estos movimientos. El programa tras calcular la IK, calcula las fuerzas externas
% que se aplican sobre el brazo del paciente, para, a continuaciï¿½n, realizar un anï¿½lisis CMC,
% que devolverï¿½ los valores de excitationes y activaciones. Tras este cï¿½lculo, se ejecuta una
% ï¿½ltima herramienta (Forward Dynamics) que calcula el resto de valores necesarios para la investigaciï¿½n.
% Adicionalmente el programa ejecuta una
% dinamica inversa a partir del archivo .mot, a la que se le podrï¿½a sumar
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
trayAnalisis='completa'; %'bajada' 'completa' 'subida'
methodIIWA_FD= 'Screw Theory'; %'Matlab toolbox'   Screw Theory
modeladoMuscular='Sano_Millard'; %'Spastic_Millard'   Sano_Thelen   Spastic_Thelen   Sano_Millard
oposicionMov='Sin fuerza'; % Sin fuerza    Con fuerza
grado=4; % Regresiï¿½n fuerzas Screw Theory
version='V1'; % V2 %'V1' para usar el modelo de Eduardo, 'V2' para usar el modelo de Anaelle

file_path = which(mfilename);
id_ch_folders = find(file_path =='\');
pathOpenSimControl = file_path(1:id_ch_folders(end-2));
disp(['Using OpenSimControl path: ', pathOpenSimControl]);

pathOpenSim = ['C:\OpenSim ', char(org.opensim.modeling.opensimCommon.GetVersion())];
% title=strcat('Selecciona el directorio de instalaciï¿½n de OpenSim') ;
% pathOpenSim = uigetdir(pathOpenSim, title);
% if (pathOpenSim == 0)
%     ME = MException('Main:NoOpenSimPath', 'No OpenSim path selected');
%     throw(ME);
% end
% disp(['Using OpenSim installation path: ', pathOpenSim]);

% Path trayectorias Iiwa
IIWADataPath=[pathOpenSimControl, '\TrayectoriasGrabadas\Test-1707\iiwa1707'];

% Path model folder
CD_model= [pathOpenSimControl, '\ROBOESPAS_FLEXION'];
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

model = f_ModelCoordChanges(CD_model,MODELO);
model.print(strcat(CD_model,'\',MODELO))

% Load Iiwa trajectory
if isequal(oposicionMov,'Con fuerza')
    Dsalida = load ([IIWADataPath, '\confuerza.mat']);
    Datos=Dsalida.AVRconfuerza;
else
    Dsalida = load ([IIWADataPath, '\sinfuerza.mat']);
    Datos=Dsalida.AVRsinfuerza;
end
DatosVacio=Dsalida.vacio;

% Load IIWA-FT data
load(strcat(pathOpenSimControl, 'TrayectoriasGrabadas\pruebaIIWA_FT.mat'));
t = Datos{1}.stamps;
tsample = t(2) - t(1);
dataSize = length(t)-1;
% Datos = {Datos};
% DatosVacio = {DatosVacio};
%% Introduccion y adecuacion de los datos del laboratorio
% Obtenciï¿½n de la trayectoria del Handle del IIWA a travï¿½s de FK
FKHandle = FK(Datos{1}.trayectoria)';
CMarkers.Handle = f_HandleCoordModifications(FKHandle(:,1:3), model);
V_IIWA = CMarkers.Handle;
% t = Datos{1}.stamps;
% tsample = t(2) - t(1);
% dataSize = length(Datos{1}.stamps)-1;

% Una vez cortados y equidistanciados, corregimos el sistema de
% coordenadas del IIWA
% [CMarkers.Handle] = f_CoordModifications(CMarkers, 'IIWA');
% Modificaciï¿½n de los datos de fuerzas y momentos (para reducir tiempos de
% computaciï¿½n en las pruebas) OPCIONAL
% FuerzasIIWA = f_ForcesModifications(Datos);

% Creo una trcTable nueva
[TrcTableCreada] = f_CreateTRCTable(t,CMarkers,'IIWA');

% Lo imprimo en un archivo .trc (Coordenadas cartesianas espaciales)

%cd(CD_trc)
CD_trc=strcat(CD_model,'\CCartesianas');
org.opensim.modeling.TRCFileAdapter.write(TrcTableCreada,[CD_trc, '\Lab.trc']);
pause(0.2);

%Variables para compensar los valores de los momentos:
V_OpenSim=CMarkers.Handle;

clear Dsalida CMarkers jp_Handle KinectData

%% Modificaciones en el modelo previas a su uso (posición por defecto, coords. bloqueadas, rango de mvto...)

% model = f_ModelCoordChanges(CD_model,MODELO);
% model.print(strcat(CD_model,'\',MODELO))

% Creo los archivos txt para almacenar las velocidades de fibra en los
% mï¿½sculos espasticos
numMuscles=model.getMuscles.getSize;
for i=0:numMuscles-1
    if model.getMuscles.get(i).hasProperty('gain_factor') %Si tiene la propiedad gain_factor significa que es espï¿½stico
        spasticMuscleName = char(model.getMuscles.get(i).getName);
        fid = fopen( strcat(pathOpenSim,'\bin\',spasticMuscleName,'_previousFiberVelocities.txt'), 'wt' );
        fidMatlab = fopen( strcat(matlabroot,'\bin\win64\',spasticMuscleName,'_previousFiberVelocities.txt'), 'wt' );
        fclose(fid);
        fclose(fidMatlab);
    end
end

%     clear numMuscles spasticMuscleName fid fidMatlab
pause(0.2);

%% CINEMATICA INVERSA
OutputMotionStr = 'Movimiento.mot';
CD_CArticulares=strcat(CD_model,'\CArticulares');
[StartTime,LastTime]=fCinInv(CD_trc,CD_CArticulares,model,'Lab.trc',OutputMotionStr);
motFilePath=strcat(CD_CArticulares,'\',OutputMotionStr);
pause(2);


%% Obtenciï¿½n de las fuerzas y momentos en el TCP
%cd(CD_model);
if isequal(methodIIWA_FD,'Screw Theory')
   [ForceAndTorque,stampsST] = f_IIWA_FD(Datos,DatosVacio,tsample); % SCREW THEORY

   % Limpio la grï¿½fica por minimos cuadrados
   % Fuerza en X
    x=stampsST(1:end-1);
    y=ForceAndTorque(:,1)';
    px = polyfit(x,y,grado);
    ForceAndTorqueLimpio(:,1) = polyval(px,x)';
    % Fuerza en Y
    y=ForceAndTorque(:,2)';
    py = polyfit(x,y,grado);
    ForceAndTorqueLimpio(:,2) = polyval(py,x)';
    % Fuerza en Z
    y=ForceAndTorque(:,3)';
    pz = polyfit(x,y,grado);
    ForceAndTorqueLimpio(:,3) = polyval(pz,x)';
    % Momento en X
    x=stampsST(1:end-1);
    y=ForceAndTorque(:,7)';
    px = polyfit(x,y,grado);
    ForceAndTorqueLimpio(:,7) = polyval(px,x)';
    % Momento en Y
    y=ForceAndTorque(:,8)';
    py = polyfit(x,y,grado);
    ForceAndTorqueLimpio(:,8) = polyval(py,x)';
    % Momento en Z
    y=ForceAndTorque(:,9)';
    pz = polyfit(x,y,grado);
    ForceAndTorqueLimpio(:,9) = polyval(pz,x)';
    ForceAndTorque=ForceAndTorqueLimpio;
    clear x y px py pz ForceAndTorqueLimpio

    % Cambio el sentido de todos los vectores para que actuen sobre el brazo
    % del paciente
    Fx=-ForceAndTorque(:,1);
    Fy=-ForceAndTorque(:,2);
    Fz=-ForceAndTorque(:,3);

    Tx=ForceAndTorque(:,7); % El sentido de los torques se trabaja a posteriori
    Ty=ForceAndTorque(:,8);
    Tz=ForceAndTorque(:,9);

    ForceAndTorque(:,1)= -Fy;
    ForceAndTorque(:,2)= Fz;
    ForceAndTorque(:,3)= -Fx;

    ForceAndTorque(:,4)= 0;
    ForceAndTorque(:,5)= -0.08;
    ForceAndTorque(:,6)= 0;

    % Compensar longitudes (brazos) de los momentos
    for i=1:dataSize
    M_OpenSim(i,1)=(norm(V_OpenSim(i,2:3))/norm(V_IIWA(i,2:3)))*Tx(i);
    M_OpenSim(i,2)=(norm(V_OpenSim(i,1:2:3))/norm(V_IIWA(i,1:2)))*Tz(i);
    M_OpenSim(i,3)=(norm(V_OpenSim(i,1:2))/norm(V_IIWA(i,1:2:3)))*Ty(i);
    end

    ForceAndTorque(:,7)=M_OpenSim(:,1);
    ForceAndTorque(:,8)=M_OpenSim(:,2);
    ForceAndTorque(:,9)=M_OpenSim(:,3);

else
    % Calculo FD haciendo uso de la toolbox de Matlab, EN EL SIST COORD DEL ROBOT
    [ForceAndTorque] = f_TCPForces(Datos,DatosVacio,tsample);

    % Cambio el sentido de todos los vectores para que actuen sobre el brazo
    % del paciente
    Fx=-ForceAndTorque(:,1);
    Fy=-ForceAndTorque(:,2);
    Fz=-ForceAndTorque(:,3);

    Tx=ForceAndTorque(:,7); % El sentido de los torques se trabaja a posteriori
    Ty=ForceAndTorque(:,8);
    Tz=ForceAndTorque(:,9);

    % salen con la siguiente orientaciï¿½n de la Toolbox:
    %
    %               |----->z
    %              /|
    %             /x|y
    % orientacion de OpenSim
    %                           Quedando:
    %                  |y                Xos=-Z
    %                  | /z              Yos=-y
    %           x<-----|/                Zos=-x

    ForceAndTorque(:,1)= -1.*(Fz);%-Fy(1));
    ForceAndTorque(:,2)=-1.*(Fy);%-Fz(1));
    ForceAndTorque(:,3)= -1.*(Fx);%-Fx(1));

    ForceAndTorque(:,4)= 0;
    ForceAndTorque(:,5)= -0.08;
    ForceAndTorque(:,6)= 0;

    ForceAndTorque(:,7)= -0.001.*(Tz);%-Ty(1));
    ForceAndTorque(:,8)= -0.001.*(Ty);%-Tz(1));
    ForceAndTorque(:,9)= 0.*-1.*(Tx);%-Tx(1)); % la componente z me da igual porque el propio mango ya resbala
end

    clear Fx Fy Fz Tx Ty Tz
    %% Creo variable External Forces (FORCES AND TORQUES)

    %Guardo todos los datos en un Storage -> ExternalForcesStorage
    ExternalForcesTorquesStorage=org.opensim.modeling.Storage();
    ExternalForcesTorquesStorage.setName('ExternalLoads.mot');   %
    ExternalForcesTorquesStorage.setInDegrees(true); %Igual que en Gait2354
    % Time=TrcTableCreada.getIndependentColumn;
    %     Force=ForceAndTorque(:,1:3);
    %     Torque=ForceAndTorque(:,7:9);
    ColumnLabels=org.opensim.modeling.ArrayStr();
    ColumnLabels.append('time');
    ColumnLabels.append('hand_force_vx');
    ColumnLabels.append('hand_force_vy');
    ColumnLabels.append('hand_force_vz');
    ColumnLabels.append('hand_force_px');
    ColumnLabels.append('hand_force_py');
    ColumnLabels.append('hand_force_pz');
    ColumnLabels.append('hand_torque_x');
    ColumnLabels.append('hand_torque_y');
    ColumnLabels.append('hand_torque_z');

    ExternalForcesTorquesStorage.setColumnLabels(ColumnLabels);
    %Meto los valores COMO STATEVECTORS
    for i=1:length(t(1,:))-1
        fila=org.opensim.modeling.StateVector();
        v=org.opensim.modeling.Vector();
        v.resize(int16(length(ForceAndTorque(1,:))));
        for j = 1:length(ForceAndTorque(1,:))
            v.set(int16(j-1),ForceAndTorque(i,j)); %Vector empieza desde 0
            fila.setStates(1,v);
            fila.setTime(t(i));
            %             size=fila.getSize;
        end
        ExternalForcesTorquesStorage.append(fila);
    end

    ExternalForcesTorquesStorage.print(strcat(CD_model,'\ForcesAndTorques\ExternalLoads.mot'));

    External_Force_FT=org.opensim.modeling.ExternalForce(ExternalForcesTorquesStorage,'hand_force_v','hand_force_p','hand_torque_','hand','ground','hand');

    External_Force_FT.setName('TCP_ExternalForce');
    External_Force_FT.setAppliedToBodyName('hand');        %%%%%
    % External_Force.setForceExpressedInBodyName('hand');  % No aplica en
    % el caso de que la fuerza se aplique sobre un body (Instrucciones de
    % OpenSim)
    External_Force_FT.print(strcat(CD_model,'\ForcesAndTorques\ExternalForce.xml'));

    clear ColumnLabels fila v
    %% Aplico las fuerzas en External Loads (FORCES AND TORQUES)

    External_Loads_FT=org.opensim.modeling.ExternalLoads();
    External_Loads_FT.adoptAndAppend(External_Force_FT);
    External_Loads_FT.setLowpassCutoffFrequencyForLoadKinematics(6);
    External_Loads_FT.setDataFileName('ExternalLoads.mot');
    % External_Loads_FT.setExternalLoadsModelKinematicsFileName(motFilePath); NO ES NECESARIO

    xmlExternalLoadsFileName_FT=strcat(CD_model,'\ForcesAndTorques\ExternalLoads.xml');
    External_Loads_FT.print(xmlExternalLoadsFileName_FT);
    motExternalLoadsFileName_FT=strcat(CD_model,'\ForcesAndTorques\ExternalLoads.mot');


% clear External_Force_FT External_Loads_FT
pause(1);

%% RRA - PENDIENTE DE VALIDAR
% import org.opensim.modeling.*
% CD_rra=strcat(CD_model,'\RRA');
%     rraTool=RRATool();
%
%     rraTool.setDesiredKinematicsFileName(motFilePath);
%     rraTool.setTaskSetFileName(strcat(CD_rra,'\RRA_Tasks_Roboespas_flex.xml'));
% %     rraTool.setExternalLoads(External_Loads);
%     rraTool.setExternalLoadsFileName(xmlExternalLoadsFileName_FT);  %Se queja luego si no lo meto
%     rraTool.setModel(model);
%     %Actuators
% %     rraTool.setForceSetFiles(strcat(CD_rra,'Reserve_Actuators_Roboespas.xml')); %%%%%%%
%     rraTool.setResultsDir(CD_rra);
% rraTool.run
% disp('RRA done');
%% TASKS - Cï¿½digo de referencia para futuros usos
% NOTA: Este archivo ya se suministra junto con el resto, el cï¿½digo sirve
% para crear automï¿½ticamente el archivo xml en el caso de querer
% modificarlo.
% %Marker Tasks
%     %Handle
%     Handle_Task=org.opensim.modeling.CMC_Task(1,true);
%     Handle_Task.setWeight(2);
%     Handle_Task.setName('Handle_Task');
%     %Cuello
%     Cuello_Task=org.opensim.modeling.IKTask(1,true);
%     Cuello_Task.setName('Cuello_Task');
% Marker_Tasks=org.opensim.modeling.IKTaskSet('Marker_Tasks');
% Marker_Tasks.print('D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M - OS40\PRUEBAS\Markertasks.xml');


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
%Bloqueos:
elv_angle.set_locked(0);
shoulder_elv.set_locked(0);
shoulder_rot.set_locked(0);
elbow_flexion.set_locked(0);
pro_sup.set_locked(1);
deviation.set_locked(1);
flexion.set_locked(1);
model.print(strcat(CD_model,'\',MODELO));

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
cmcTool.setExternalLoadsFileName(xmlExternalLoadsFileName_FT);
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
% NO TIENE BIEN DEFINIDAS LAS DINï¿½MICAS. SI EN UN FUTURO SE DISPONE DE OTRO
% MODELO QUE CUMPLA ESTO, O SE CONSIGUEN MEJORAR ESTAS DINï¿½MICAS, SE DEBERï¿½
% USAR LA OPCIï¿½N FastTarget
cmcTool.setUseFastTarget(false);

% % Points: No es necesario
% %   cmcTool.setDesiredPointsFileName(motFilePath);

cmcTool.print(strcat(CD_cmc,'\setupActualCMC.xml'));

cmc = CMCTool(strcat(CD_cmc,'\setupActualCMC.xml'));
cmc.run; %Esto se hace por seguridad. Recomendaciï¿½n de OpenSim.
% cmcTool.run;

disp('CMC Completado');

%% Representar excitaciones estimadas con CMC
[cmcExcitations] = f_importCMCResults(strcat(CD_cmc, 'Results\CMCtool_controls.sto'));
[cmcActivations] = f_importCMCResults(strcat(CD_cmc, 'Results\CMCtool_states.sto'));
muscles = fieldnames(cmcExcitations);
numMuscles = length(muscles)-1;
f = figure('Name', 'Estimated EMG', 'WindowState', 'maximized');
for i = 1:numMuscles
    subplot(numMuscles,1,i, 'Parent',f);
    hold on
    plot(cmcExcitations.time, cmcExcitations.(muscles{i+1}))
    plot(cmcActivations.time, cmcActivations.(muscles{i+1}), 'Color', 'r')
    title(muscles{i+1})
    legend('Excitación', 'Activación')
    ylim([0 1])
    drawnow
end
%% Dinï¿½mica directa a partir de las external loads

[FDOutputPath] = f_FD(model,ExternalForcesTorquesStorage,'',CD_model,External_Loads_FT);

pause(0.2);


% -------------------------------------------------------------------
% OPCIONALES: FUNCIONAN, SIMPLEMENTE DESCOMENTAR
% -------------------------------------------------------------------
%% REPRODUCCION DEL MOVIMIENTO
Motion="Movimiento.mot";
% f_PlayVisualizer(CD_CArticulares,model,Motion)

%% Realizo un MUSCLE ANALYSIS sobre el .mot para obtener velocidades
% cd(CD_model)
% motPath=strcat(CD_CArticulares,'\',OutputMotionStr); %Path del .mot
% ARPath=strcat(CD_model,'\AnalizeResults');
% MAResults=f_AnalyzeTool(CD_model,model,motPath,ARPath);
%
% %% DINAMICA INVERSA
% OutputIDStr = 'IDoutput.sto';
%
% % EndTime = length(x);
% CD_IDOutput=strcat(CD_model,'\ID_Output');
% % f_ID(CD_IDOutput,model,strcat(CD_CArticulares,'\',OutputMotionStr),OutputIDStr,StartTime,LastTime);