

for iiwa_files=0:10

clearvars -except iiwa_files
    
import org.opensim.modeling.*
format long
pause(2);
%% INPUTS

%   <<<<<<<<<<<<<MODIFICABLES:>>>>>>>>>>>>
% Masa del sujeto
masaTotal=67;
mass=4.77882; %4.77882 es la que tiene el modelo por defecto
%masaTotal*0.55; % la masa del tronco superior es aproximadamente el 55% de la masa  total

path='C:\Users\gvalesquez\Documents\MATLAB\Archivos_EduardoSaenz\ROBOESPAS_FLEXION\Geometry';
ModelVisualizer.addDirToGeometrySearchPaths(path);


methodIIWA_FD= 'Screw Theory';
dataUsed='IIWA_Kinect'; %'Kinect' 'IIWA_Kinect' 'IIWA'
oposicionMov='Sin fuerza'; % Sin fuerza    Con fuerza 
grado=4; % Regresión fuerzas Screw Theory
     

%Load spasticMillardMuscleModel


opensimCommon.LoadOpenSimLibrary("..\Plugins\SpasticMillardMuscleModel.dll")
MODELO= 'Arm_Flexion_SpasticMillard.osim';
%MODELO='Arm_Flexion_Millard.osim';


%IIWA:

IIWADataPath='C:\Users\gvalesquez\Documents\MATLAB\Archivos_EduardoSaenz\TrayectoriasGrabadas\Test-1707\iiwa1707';
cd(IIWADataPath);


Dsalida = load ('sinfuerza.mat');

Data_IIWA_Vacio = Dsalida.vacio;
Data_IIWA{1} = Dsalida.AVRsinfuerza; 
Data_IIWA{2} = Dsalida.sinfuerza01;
Data_IIWA{3} = Dsalida.sinfuerza02;
Data_IIWA{4} = Dsalida.sinfuerza03;
Data_IIWA{5} = Dsalida.sinfuerza04;
Data_IIWA{6} = Dsalida.sinfuerza05;
Data_IIWA{7} = Dsalida.sinfuerza06;
Data_IIWA{8} = Dsalida.sinfuerza07;
Data_IIWA{9} = Dsalida.sinfuerza08;
Data_IIWA{10} = Dsalida.sinfuerza09;
Data_IIWA{11} = Dsalida.sinfuerza10;
%Datos=Dsalida.AVRsinfuerza;
%Datos=Dsalida.sinfuerza08;
   
    
%DatosVacio=Dsalida.vacio;


CD_Matlab = 'C:\Program Files\MATLAB\R2019a';
CD_Opensim = 'C:\OpenSim 4.0';

CD_model='C:\Users\gvalesquez\Documents\MATLAB\Archivos_EduardoSaenz\ROBOESPAS_FLEXION';


%Kinect:
KinectFrequency = 30; %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
tsample=1/KinectFrequency;
   

KinectFilepath = "C:\Users\gvalesquez\Documents\MATLAB\Archivos_EduardoSaenz\TrayectoriasGrabadas\Test-1707\TrayectoriasKinect1707\Sin fuerza";

KinectFile='Tray7';
KinectFilename = "Tray7.csv";%"Trayectoria Excel.csv";


KinectData=importdata(strcat(KinectFilepath,'\',KinectFilename));
KinectStartRow=3;
KinectEndRow=size(KinectData,1); 

g_labels= {'0'; '1'; '1.1'; '2'; '3'; '4'};
Ganancia = 1;

    
    
    
    
    
    
   %clear cmcTool cmc sto xmlExternalLoadsFileName_FT model V_OpenSim V_IIWA TrcTableCreada trayAnalisis tk tI t StartTime stampsST stamps OutputMotionStr numMuscles motFilePath motExternalLoadsFileName_FT  margin M_OpenSim LastTime initialValueZ initialValueY initialValueX ForceAndTorque FKHandle finalValueZ finalValueY finalValueX ExternalForcesTorquesStorage External_Loads_FT External_Force_FT DatosVacio Datos;
   % clear CMarkers FKHandle jp_Handle;
    pause(10);
    Datos = Data_IIWA{iiwa_files + 1};
    
    DatosVacio=Data_IIWA_Vacio;
    
    
    CMarkers = f_CSVreader(KinectFilepath,KinectFilename,KinectStartRow,KinectEndRow);

    
    


for i = 0:(KinectEndRow-KinectStartRow)
   tk(i+1)= tsample*i;
end  
 

% Recorto datos de kinect e iiwa para que empiecen y acaben sincronizados:
 %   Para ello, me fijo en todas las componentes de WristRight, que es el marker
 %   más fiable a la hora de determinar cuando empieza el movimiento.
% KINECT
 initialValueX=CMarkers.WristRight(1,1);
 finalValueX=CMarkers.WristRight(end,1);
 initialValueY=CMarkers.WristRight(1,2);
 finalValueY=CMarkers.WristRight(end,2);
 initialValueZ=CMarkers.WristRight(1,3);
 finalValueZ=CMarkers.WristRight(end,3);
 j=0;
 k=length(CMarkers.WristRight(:,1));
 AcotadoInferior=0;
 AcotadoSuperior=0;
 margin=0.01;
 for i =1:length(CMarkers.WristRight(:,2))
     if (abs(CMarkers.WristRight(i,1)-initialValueX)>margin &&  abs(CMarkers.WristRight(i,2)-initialValueY)>margin ...
             && abs(CMarkers.WristRight(i,3)-initialValueZ)>margin && AcotadoInferior==0)
         %Ha empezado el movimiento en el frame j
         j=i;
         AcotadoInferior=1;
     end
     if (abs(CMarkers.WristRight(i,1)-finalValueX)<margin &&  abs(CMarkers.WristRight(i,2)-finalValueY)<margin ...
             && abs(CMarkers.WristRight(i,3)-finalValueZ)<margin && AcotadoSuperior==0 && AcotadoInferior==1)
         %Ha acabado el movimiento en el frame k
         k=i;
         AcotadoSuperior=1;
     end
 end

 %Antes de recortar la trayectoria voy a utilizar los primeros
 %valores de la Kinect para hacer un escalado del modelo
 CD_trc=strcat(CD_model,'\CCartesianas');
 
 %Escalado!!!
 %if escalarModelo==true
 %scaleLastTime=j*tsample;
 %[model,MODELO] = f_scaleModel(MODELO,scaleLastTime,mass,CD_model);
 %end

 CMarkers.SpineBase= CMarkers.SpineBase(j:k,:);
 CMarkers.SpineMid=CMarkers.SpineMid(j:k,:);
 CMarkers.Neck=CMarkers.Neck(j:k,:);
 CMarkers.Head=CMarkers.Head(j:k,:);
 CMarkers.ShoulderRight=CMarkers.ShoulderRight(j:k,:);
 CMarkers.ElbowRight=CMarkers.ElbowRight(j:k,:);
 CMarkers.WristRight=CMarkers.WristRight(j:k,:);
 CMarkers.HandRight=CMarkers.HandRight(j:k,:);
 CMarkers.HipLeft=CMarkers.HipLeft(j:k,:);
 CMarkers.HipRight=CMarkers.HipRight(j:k,:);
 CMarkers.SpineShoulder=CMarkers.SpineShoulder(j:k,:);
 CMarkers.HandTipRight=CMarkers.HandTipRight(j:k,:);
 CMarkers.ThumbRight=CMarkers.ThumbRight(j:k,:);
 tk=tk(j:k)-tk(j);

 % IIWA
 % Equidisto los puntos de Handle para que cuadren con la kinect

[stamps, jp_Handle]=equidistant(Datos{1,1}.stamps, Datos{1,1}.trayectoria, tsample);
FKHandle = FK(jp_Handle);
 CMarkers.Handle=FKHandle(1:3,:)';
 % Acoto los datos del IIWA basandome en la misma idea pero en este caso
 % utilizando la posición del endeffector
 initialValueX=CMarkers.Handle(1,1);
 finalValueX=CMarkers.Handle(end,1);
 initialValueY=CMarkers.Handle(1,2);
 finalValueY=CMarkers.Handle(end,2);
 initialValueZ=CMarkers.Handle(1,3);
 finalValueZ=CMarkers.Handle(end,3);
 j=0;
 k=length(CMarkers.Handle(:,1));
 AcotadoInferior=0;
 AcotadoSuperior=0;
 margin=0.01;
 for i =1:length(CMarkers.Handle(:,2))
     if (abs(CMarkers.Handle(i,1)-initialValueX)>margin &&  abs(CMarkers.Handle(i,2)-initialValueY)>margin ...
             && abs(CMarkers.Handle(i,3)-initialValueZ)>margin && AcotadoInferior==0)
         %Ha empezado el movimiento en el frame j
         j=i;
         AcotadoInferior=1;
     end
     if (abs(CMarkers.Handle(i,1)-finalValueX)<margin &&  abs(CMarkers.Handle(i,2)-finalValueY)<margin ...
             && abs(CMarkers.Handle(i,3)-finalValueZ)<margin && AcotadoSuperior==0 && AcotadoInferior==1)
         %Ha acabado el movimiento en el frame k
         k=i;
         AcotadoSuperior=1;
     end
 end
CMarkers.Handle= CMarkers.Handle(j:k,:);
V_IIWA=CMarkers.Handle;
tI=stamps(j:k)-stamps(j);
    


% Hasta este punto los datos de IIWA y Kinect llegan a la misma frecuencia
% y empezando y acabando aproximadamente en el mismo instante. No obstante,
% lo mas probable es que un vector sea de mayor longitud que el otro, por
% lo que recorto el mayor para poder introducir estos datos en TRC.
dataSize=min(size(tI,2),size(tk,2));
t=tI(1:dataSize);
 CMarkers.SpineBase= CMarkers.SpineBase(1:dataSize,:);
 CMarkers.SpineMid=CMarkers.SpineMid(1:dataSize,:);
 CMarkers.Neck=CMarkers.Neck(1:dataSize,:);
 CMarkers.Head=CMarkers.Head(1:dataSize,:);
 CMarkers.ShoulderRight=CMarkers.ShoulderRight(1:dataSize,:);
 CMarkers.ElbowRight=CMarkers.ElbowRight(1:dataSize,:);
 CMarkers.WristRight=CMarkers.WristRight(1:dataSize,:);
 CMarkers.HandRight=CMarkers.HandRight(1:dataSize,:);
 CMarkers.HipLeft=CMarkers.HipLeft(1:dataSize,:);
 CMarkers.HipRight=CMarkers.HipRight(1:dataSize,:);
 CMarkers.SpineShoulder=CMarkers.SpineShoulder(1:dataSize,:);
 CMarkers.HandTipRight=CMarkers.HandTipRight(1:dataSize,:);
 CMarkers.ThumbRight=CMarkers.ThumbRight(1:dataSize,:);
 CMarkers.Handle=CMarkers.Handle(1:dataSize,:);
 V_IIWA=V_IIWA(1:dataSize,:);

 % Una vez cortados y equidistanciados, corregimos el sistema de
% coordenadas del IIWA
 [CMarkers.Handle] = f_CoordModifications(CMarkers); 
% Modificación de los datos de fuerzas y momentos (para reducir tiempos de
% computación en las pruebas) OPCIONAL
% FuerzasIIWA = f_ForcesModifications(Datos);


% Creo una trcTable nueva
[TrcTableCreada] = f_CreateTRCTable(t,CMarkers,dataUsed);

% Lo imprimo en un archivo .trc (Coordenadas cartesianas espaciales)

cd(CD_trc)
org.opensim.modeling.TRCFileAdapter.write(TrcTableCreada,'Lab.trc')
pause(0.2);

%Variables para compensar los valores de los momentos:
% V_IIWA=FKHandle(1:3,:)'; %Posición desde IIWA ya equidistado
V_OpenSim=CMarkers.Handle;
% clear AcotadoInferior AcotadoSuperior tI tk TrcTableCreada
clear CMarkers jp_Handle KinectData  %Dsalida
% clear finalValueX finalValueY finalValueX initialValueX initialValueY initialValueZ 
%% Modificaciones en el modelo previas a su uso

model = f_ModelCoordChanges(CD_model,MODELO);
model.print(strcat(CD_model,'\',MODELO)) 

% Creo los archivos txt para almacenar las velocidades de fibra en los
% músculos espasticos
    numMuscles=model.getMuscles.getSize;
    for i=0:numMuscles-1
       if model.getMuscles.get(i).hasProperty('gain_factor') %Si tiene la propiedad gain_factor significa que es espástico
           spasticMuscleName = char(model.getMuscles.get(i).getName);
           fid = fopen( strcat(CD_Opensim,'\bin\',spasticMuscleName,'_previousFiberVelocities.txt'), 'wt' );
           fidMatlab = fopen( strcat(CD_Matlab,'\bin\win64\',spasticMuscleName,'_previousFiberVelocities.txt'), 'wt' );
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


%% Obtención de las fuerzas y momentos en el TCP
cd(CD_model);

   [ForceAndTorque,stampsST] = f_IIWA_FD(Datos,DatosVacio,tsample); % SCREW THEORY
   
   % Limpio la gráfica por minimos cuadrados
   % Fuerza en X
   if iiwa_files == 0
    x=stampsST(1:end-1); %x=stampsST(1:end-1);
   else
    x=stampsST(1:end);
   end
   
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
    if iiwa_files == 0
    x=stampsST(1:end-1); %x=stampsST(1:end-1);
   else
    x=stampsST(1:end);
   end
    %x=stampsST(1:end); %x=stampsST(1:end-1);
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

 ForceAndTorque=ForceAndTorque(j:j+dataSize-1,:);    

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
for i=1:length(t(1,:))   
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
   
%% CMC 



%return;
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
    cmcTool.setResultsDir(strcat(CD_model,'\CMCResults\G1\TrayIIWA',string(iiwa_files)));
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
    cmcTool.setMaxDT(1e-01);
% Minimum integration step size
    cmcTool.setMinDT(1e-02);    
% Integrator error tolerance
    cmcTool.setErrorTolerance(1e-03);
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
% NO TIENE BIEN DEFINIDAS LAS DINÁMICAS. SI EN UN FUTURO SE DISPONE DE OTRO
% MODELO QUE CUMPLA ESTO, O SE CONSIGUEN MEJORAR ESTAS DINÁMICAS, SE DEBERÁ
% USAR LA OPCIÓN FastTarget
    cmcTool.setUseFastTarget(false);
    
% % Points: No es necesario
% %   cmcTool.setDesiredPointsFileName(motFilePath);   

cmcTool.print(strcat(CD_cmc,'\setupActualCMC.xml'));

cmc = CMCTool(strcat(CD_cmc,'\setupActualCMC.xml'));
cmc.run; %Esto se hace por seguridad. Recomendación de OpenSim.
% cmcTool.run;

disp('CMC Completado');
%% Dinámica directa a partir de las external loads

%[FDOutputPath] = f_FD(model,ExternalForcesTorquesStorage,'',CD_model,xmlExternalLoadsFileName_FT);
 %[FDOutputPath] = f_FD(model,ExternalForcesTorquesStorage,'',CD_model,External_Loads_FT);
%motExternalLoadsFileName_FT   xmlExternalLoadsFileName_FT
pause(0.2);




end  %end for para crear diferentes cmc con mismo tray kinect, variando iiwa
