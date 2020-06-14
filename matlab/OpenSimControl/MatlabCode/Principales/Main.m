% Este programa lee los vectores de posición y fuerzas de la herramienta del robot en
% un archivo .mat y los archivos proporcionados por la Kinect en un archivo CSV. 
% Transforma estas tablas en una tabla .trc para que pueda
% ser utilizada por OpenSim y realizar una cinemática inversa. Una vez
% obtenido el archivo .mot, mediante IK, con el registro de los movimientos de cada
% coordenada del modelo, se puede lanzar un visualizador para que
% represente estos movimientos. El programa tras calcular la IK, calcula las fuerzas externas
% que se aplican sobre el brazo del paciente, para, a continuación, realizar un análisis CMC,
% que devolverá los valores de excitationes y activaciones. Tras este cálculo, se ejecuta una 
% última herramienta (Forward Dynamics) que calcula el resto de valores necesarios para la investigación.
% Adicionalmente el programa ejecuta una
% dinamica inversa a partir del archivo .mot, a la que se le podría sumar
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
mass=4.77882; %4.77882 es la que tiene el modelo por defecto
%masaTotal*0.55; % la masa del tronco superior es aproximadamente el 55% de la masa  total

escalarModelo=false;
trayAnalisis='completa'; %'bajada' 'completa' 'subida'
methodIIWA_FD= 'Screw Theory'; %'Matlab toolbox'   Screw Theory
modeladoMuscular='Sano_Millard'; %'Spastic_Millard'   Sano_Thelen   Spastic_Thelen   Sano_Millard
dataUsed='IIWA_Kinect'; %'Kinect' 'IIWA_Kinect' 'IIWA'
oposicionMov='Sin fuerza'; % Sin fuerza    Con fuerza 
grado=4; % Regresión fuerzas Screw Theory
switch modeladoMuscular
    case 'Spastic_Thelen'
        %Load spasticMuscleModelPlugin
        opensimCommon.LoadOpenSimLibrary("..\Plugins\SpasticThelenMuscleModel.dll")
        MODELO= 'Arm_Flexion_SpasticThelen.osim';
    case 'Spastic_Millard'
        %Load spasticMillardMuscleModel
        opensimCommon.LoadOpenSimLibrary("..\Plugins\SpasticMillardMuscleModel.dll")
        MODELO= 'Arm_Flexion_SpasticMillard.osim';
    case 'Sano_Thelen'
        MODELO='Arm_Flexion_Thelen.osim';
    case 'Sano_Millard'
        MODELO='Arm_Flexion_Millard.osim';
end


    %IIWA:
    IIWADataPath='D:\Leytha\Documentos\GitHub\roboespas\matlab\OpenSimControl\TrayectoriasGrabadas\Test-1707\iiwa1707';
    cd(IIWADataPath);
    if isequal(oposicionMov,'Con fuerza')
        Dsalida = load ('confuerza.mat');
        Datos=Dsalida.AVRconfuerza;
    else
        Dsalida = load ('sinfuerza.mat');
        Datos=Dsalida.AVRsinfuerza;
    end
    
    DatosVacio=Dsalida.vacio;
    
    CD_Matlab = 'D:\User\D_Software\Universidad\Matlab UC3M';
    CD_Opensim = 'D:\User\D_Software\Universidad\OpenSim 4.0';
    CD_model='D:\Leytha\Documentos\GitHub\roboespas\matlab\OpenSimControl\ROBOESPAS_FLEXION';
    
    %Kinect:
    KinectFrequency = 30; %!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    tsample=1/KinectFrequency;
    if isequal(oposicionMov,'Sin fuerza')
    KinectFilepath = "D:\Leytha\Documentos\GitHub\roboespas\matlab\OpenSimControl\TrayectoriasGrabadas\Test-1707\TrayectoriasKinect1707\Sin fuerza";
    else
    KinectFilepath = "D:\Leytha\Documentos\GitHub\roboespas\matlab\OpenSimControl\TrayectoriasGrabadas\Test-1707\TrayectoriasKinect1707\Con fuerza";
    end
    KinectFilename = "Tray7.csv";%"Trayectoria Excel.csv";
    KinectData=importdata(strcat(KinectFilepath,'\',KinectFilename));
    KinectStartRow=3;
    KinectEndRow=size(KinectData,1); 
    
    %% Introduccion y adecuacion de los datos del laboratorio
 switch dataUsed
    case 'IIWA'
        % Modificaciones del sistema de coordenadas
        % requeridas para adaptar el sistema de coordenadas del laboratorio al del entorno SimTK
        [Handle] = f_CoordModifications(Datos); 
     case 'Kinect'
        % Datos recogidos por la Kinect
        CMarkers = f_CSVreader(KinectFilepath,KinectFilename,KinectStartRow,KinectEndRow);
        Handle=CMarkers.HandRight;
        
        % La kinect de momento no proporciona stamps, por lo que utilizo la
        % frecuencia leida por la kinect, no está comprobada su exactitud
        for i = 0:(KinectEndRow-KinectStartRow)
            tk(i+1)= 1/KinectFrequency*i;
        end
     case 'IIWA_Kinect'
         
         CMarkers = f_CSVreader(KinectFilepath,KinectFilename,KinectStartRow,KinectEndRow);
         for i = 0:(KinectEndRow-KinectStartRow)
            tk(i+1)= tsample*i;
         end        
         switch trayAnalisis
             case 'completa'
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
         if escalarModelo==true
         scaleLastTime=j*tsample;
         [model,MODELO] = f_scaleModel(MODELO,scaleLastTime,mass,CD_model);
         end

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
         CMarkers.Handle=FKHandle(1:3, :)';
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
        
             case 'subida'
                 % Recorto datos de kinect e iiwa para que empiecen y acaben sincronizados:
                 %en este caso solo voy a simular la trayectoria de subida
                 %de la extremidad
         %   Para ello, me fijo en la componente Y de WristRight, que es el marker
         %   más fiable a la hora de determinar cuando empieza el movimiento.
     % KINECT
         initialValueY=CMarkers.WristRight(1,2);
         finalValueY=max(CMarkers.WristRight(:,2));
         j=0;
         k=length(CMarkers.WristRight(:,1));
         AcotadoInferior=0;
         AcotadoSuperior=0;
         margin=0.05;
         for i =1:length(CMarkers.WristRight(:,2)) 
             %Acotado inicial(inferior)
             if (abs(CMarkers.WristRight(i,2)-initialValueY)>margin && AcotadoInferior==0)
                 %Ha empezado el movimiento en el frame j
                 j=i;
                 AcotadoInferior=1;
             end
             %Acotado final(superior)
             if (abs(CMarkers.WristRight(i,2)-finalValueY)<margin && AcotadoSuperior==0 && AcotadoInferior==1)
                 %Ha acabado el movimiento en el frame k
                 k=i; 
                 AcotadoSuperior=1;
             end
         end
         
         %Antes de recortar la trayectoria voy a utilizar los primeros
         %valores de la Kinect para hacer un escalado del modelo
         CD_trc=strcat(CD_model,'\CCartesianas');
         if escalarModelo==true
         scaleLastTime=j*tsample;
         [model,MODELO] = f_scaleModel(MODELO,scaleLastTime,mass,CD_model);
         end

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
         % utilizando la posición del endeffector pero para Z porque
         % todavía no esta el sist. de coordenadas adaptado a Opensim
         initialValueZ=CMarkers.Handle(1,3);
         finalValueZ=max(CMarkers.Handle(:,3));
         j=0;
         k=length(CMarkers.Handle(:,1));
         AcotadoInferior=0;
         AcotadoSuperior=0;
         margin=0.01;
         for i =1:length(CMarkers.Handle(:,2))
             if (abs(CMarkers.Handle(i,3)-initialValueZ)>margin && AcotadoInferior==0)
                 %Ha empezado el movimiento en el frame j
                 j=i;
                 AcotadoInferior=1;
             end
             if (abs(CMarkers.Handle(i,3)-finalValueZ)<margin && AcotadoSuperior==0 && AcotadoInferior==1)
                 %Ha acabado el movimiento en el frame k
                 k=i;
                 AcotadoSuperior=1;
             end
         end
        CMarkers.Handle= CMarkers.Handle(j:k,:);
        V_IIWA=CMarkers.Handle;
        tI=stamps(j:k)-stamps(j);
        
        case 'bajada'
         % Recorto datos de kinect e iiwa para que empiecen y acaben sincronizados:
         %en este caso solo voy a simular la trayectoria de subida
         %de la extremidad
         %   Para ello, me fijo en la componente Y de WristRight, que es el marker
         %   más fiable a la hora de determinar cuando empieza el movimiento.
     % KINECT
         initialValueX=max(CMarkers.WristRight(:,1));
         finalValueX=CMarkers.WristRight(end,1);
         initialValueY=max(CMarkers.WristRight(:,2));
         finalValueY=CMarkers.WristRight(end,2);
         initialValueZ=max(CMarkers.WristRight(:,3));
         finalValueZ=CMarkers.WristRight(end,3);
         j=0;
         k=length(CMarkers.WristRight(:,1));
         AcotadoInferior=0;
         AcotadoSuperior=0;
         margin=0.01;
         for i =1:length(CMarkers.WristRight(:,2)) 
             %Acotado inicial(inferior)
             if (abs(CMarkers.WristRight(i,1)-initialValueX)>margin &&  abs(CMarkers.WristRight(i,2)-initialValueY)>margin ...
                     && abs(CMarkers.WristRight(i,3)-initialValueZ)>margin && AcotadoInferior==0)
                 %Ha empezado el movimiento en el frame j
                 j=i;
                 AcotadoInferior=1;
             end
             %Acotado final(superior)
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
         if escalarModelo==true
         scaleLastTime=j*tsample;
         [model,MODELO] = f_scaleModel(MODELO,scaleLastTime,mass,CD_model);
         end

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
         initialValueX=max(CMarkers.Handle(:,1));
         finalValueX=CMarkers.Handle(end,1);
         initialValueY=max(CMarkers.Handle(:,2));
         finalValueY=CMarkers.Handle(end,2);
         initialValueZ=max(CMarkers.Handle(:,3));
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
        
        
        
        
        
        
         end %switch trayAnalisis
 end %switch dataUsed
 
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
clear Dsalida CMarkers jp_Handle KinectData  
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
if ~isequal(dataUsed,'Kinect')
if isequal(methodIIWA_FD,'Screw Theory')
   [ForceAndTorque,stampsST] = f_IIWA_FD(Datos,DatosVacio,tsample); % SCREW THEORY
   
   % Limpio la gráfica por minimos cuadrados
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
else
    % Calculo FD haciendo uso de la toolbox de Matlab, EN EL SIST COORD DEL
%     ROBOT
 [ForceAndTorque] = f_TCPForces(Datos,DatosVacio,tsample);
end
 ForceAndTorque=ForceAndTorque(j:j+dataSize-1,:);    

 % Cambio el sentido de todos los vectores para que actuen sobre el brazo
 % del paciente
Fx=-ForceAndTorque(:,1);
Fy=-ForceAndTorque(:,2);
Fz=-ForceAndTorque(:,3);

Tx=ForceAndTorque(:,7); % El sentido de los torques se trabaja a posteriori
Ty=ForceAndTorque(:,8);
Tz=ForceAndTorque(:,9);

    
if isequal(methodIIWA_FD,'Matlab toolbox')
     % salen con la siguiente orientación de la Toolbox:
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
if isequal(methodIIWA_FD,'Screw Theory')
    
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
    
    
end
% clear External_Force_FT External_Loads_FT
   pause(1);

%% Creo variable External Forces (FORCES) - OPCIÓN DESECHADA
% 
% %Guardo todos los datos en un Storage -> ExternalForcesStorage
% ExternalForcesStorage=org.opensim.modeling.Storage();
%     Time=TrcTableCreada.getIndependentColumn;
%     Force=ForceAndTorque(:,1:3);
%     ColumnLabels=org.opensim.modeling.ArrayStr();
%         ColumnLabels.append('time');
%         ColumnLabels.append('force_x');
%         ColumnLabels.append('force_y');
%         ColumnLabels.append('force_z');
%     ExternalForcesStorage.setColumnLabels(ColumnLabels);
%      %Meto los valores COMO STATEVECTORS
%     for i=1:length(Force(:,1))
%         fila=org.opensim.modeling.StateVector();
%         v=org.opensim.modeling.Vector();
%         v.resize(int16(length(Force(1,:))));
%         for j = 1:length(Force(1,:))
%                 v.set(int16(j-1),Force(i,j)); %Vector empieza desde 0
%             fila.setStates(1,v);
%             fila.setTime(Time.get(i-1));
%             size=fila.getSize;
%         end
%         ExternalForcesStorage.append(fila);
%     end
% ExternalForcesStorage.print(strcat(CD_model,'\ForcesAndTorques\ForcesAndTorquesSto.sto'));
% 
% 
% External_Force_F=org.opensim.modeling.ExternalForce(ExternalForcesStorage,'force_','','','hand','','');
% % External_Force.set_data_source_name('ForcesAndTorquesSto.sto');
% External_Force_F.setName('TCP_External_Force');
% External_Force_F.setAppliedToBodyName('hand');        %%%%%
% % External_Force.setForceExpressedInBodyName('hand');  %%%%%?
% External_Force_F.print(strcat(CD_model,'\ForcesAndTorques\ExternalForce.xml'));

%% Aplico las fuerzas en External Loads (FORCES) - OPCIÓN DESECHADA

% External_Loads_F=org.opensim.modeling.ExternalLoads();
% External_Loads_F.adoptAndAppend(External_Force_F);
% External_Loads_F.setLowpassCutoffFrequencyForLoadKinematics(6);
% External_Loads_F.setDataFileName('ForcesSto.sto');
% 
% xmlExternalLoadsFileName_F=strcat(CD_model,'\ForcesAndTorques\ExternalLoads_F.xml');
% External_Loads_F.print(xmlExternalLoadsFileName_F);
% 
% motExternalLoadsFileName_F=strcat(CD_model,'\ForcesAndTorques\ExternalLoadsMot_F.mot');
% External_Loads_F.print(motExternalLoadsFileName_F);
% pause(1);

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
 %% TASKS - Código de referencia para futuros usos
% NOTA: Este archivo ya se suministra junto con el resto, el código sirve
% para crear automáticamente el archivo xml en el caso de querer
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

%% AJUSTE ESTÁTICO DE LOS PARAMETROS MUSCULARES
% [model] = f_MuscleModelChanges(CD_model,MODELO);
% EN TRABAJOS FUTUROS ESTA FUNCIÓN SE PUEDE UTILIZAR PARA DEFINIR LOS
% VALORES DE LOS PARÁMETROS DE UN MODELO THELEN




 %% ELIJO ENTRE EL MODELO DE VELOCIDADES O EL DE FUERZAS - NO FUNCIONA
 % ESTE CÓDIGO ÚNICAMENTE SE DEJA POR SI ALGUNA INSTRUCCIÓN PUDIERA SERVIR
 % DE AYUDA, PERO NO ES FUNCIONAL. CORRESPONDE A LA SOLUCIÓN 1 DE LA
 % MEMORIA DE EDUARDO SÁENZ
% ELECCION = 4; % 1-> VELOCIDAD    2-> FUERZA    3 -> pruebas fd  4-> LONGITUD DE FIBRA
% 
% %% PRUEBA: FD CON LONGITUD DE FIBRA
% if ELECCION == 4
% 
% %     INPUTS:
% %     NormFiberLength_Path="D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M - OS40\AnalizeResults\AnalyzeResults_MuscleAnalysis_NormalizedFiberLength.sto";
%     NormFiberLength_Path="D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M - OS40\AnalizeResults\AnalyzeResults_MuscleAnalysis_NormalizedFiberLength.sto";
%     MAsto=org.opensim.modeling.Storage(NormFiberLength_Path);
%     
% % Tiempos:
%     StartTime=MAsto.getFirstTime;
%     LastTime=MAsto.getLastTime;    
%     
%     
% %   Analisis:
%     Kin=Kinematics();
%     Kin.set_model(model);
%     Kin.set_statesStore(MAsto);
% %     Kin.setStepInterval(0.001);%?
%     Kin.setStartTime(StartTime);
%     Kin.setEndTime(LastTime);
% %     Kin.setInDegrees(true);
%         model.addAnalysis(Kin)
% 
%     
% % Initialize the underlying computational system
% state = model.initSystem();
% 
% % Create the Forward Tool
% tool = ForwardTool();
% 
% % Set the model for the forward tool
% tool.setModel(model);
% 
% % Define the start and finish times 
% tool.setStartTime(StartTime);
% tool.setFinalTime(LastTime);
% 
% % Define the prefix for the result files
% tool.setName('NewFD');
% 
% % Set Input States File
% % tool.setStatesFileName("D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M - OS40\AnalizeResults\AnalyzeResults_MuscleAnalysis_FiberForce.sto");
% tool.setStatesFileName(NormFiberLength_Path);
% % Set Results Directory (will create without prompt)
% tool.setResultsDir('D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M - OS40\FD_Output');
% 
% % Run the simulation
% statusVal = tool.run();
% 
% FDOutputPath='D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M - OS40\FD_Output\NewFD_states_degrees.mot';
% 
% % Cleanup
% % clearvars walkerModel forceReporter tool state statusVal
% display('Forward Tool Finished.');
% end
% 
% %% Aplico la espasticidad
% if ELECCION == 1 || ELECCION == 3
% %INPUTS
% LimSup=1;
% LimInf=-1;
% THRESHOLD = 0.1;%REVISAR!!!!
% G=2; %GAIN
% Td=0.020; % Time delay REVISAR!!!!
% NormFVPath=strcat(MAResults,'NormFiberVelocity.sto');  
% 
% %MAIN
% [New_NFVelocity_Storage,New_NFV_Path]=f_Espasticidad(fsample,LimSup,LimInf,THRESHOLD,G,Td,NormFVPath);
% 
% % AHORA HABRIA QUE INTRODUCIR ESTOS NUEVOS DATOS EN EL MODELO PARA CERRAR
% % EL BUCLE
% disp('terminado');
% end
% 
% %% Aplico la espasticidad (MODELO DE FUERZA) 
% if ELECCION == 2
% %INPUTS
% LimSup_F=inf;
% LimInf_F=0;
% THRESHOLD_F = 0;%REVISAR!!!!
% G_F=2; %GAIN
% Td_F=0.010; % Time delay REVISAR!!!!
% FFPath=strcat(MAResults,'FiberForce.sto');
% 
% %MAIN
% [New_FForce_Storage,New_FF_Path]=f_Espasticidad_Fuerza(fsample,LimSup_F,LimInf_F,THRESHOLD_F,G_F,Td_F,FFPath);
% 
% % AHORA HABRIA QUE INTRODUCIR ESTOS NUEVOS DATOS EN EL MODELO PARA CERRAR
% % EL BUCLE
% disp('terminado');
% end
% 
% %% PRUEBA: CERRAR EL BUCLE CON MUSCLE ANALYSIS (MODELO  DE FUERZAS)
% % import org.opensim.modeling.*
% % results_folder = "D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M - OS40\PRUEBAS";
% % cd(CD_model);
% % %xml
% % genericSetupForAn = fullfile('AT_4M_Setup_Analyze_MuscleAn.xml');
% % % analyzeTool = AnalyzeTool(genericSetupForAn);
% % analyzeTool = AnalyzeTool();
% % name='CIERRE';
% % %STO
% % StoFile = "D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M - OS40\AnalizeResults\Resultados_MuscleAnalysis_NormFiberVelocity_Spastic.sto";
% % StoData = Storage(StoFile);
% %     initial_time = StoData.getFirstTime();
% %     final_time = StoData.getLastTime();
% %     
% %     
% %     % Si quiero añadir un analisis:
% %         % Kinematics
% %         Kin=Kinematics();
% %         Kin.setModel(model);
% %         Kin.set_model(model);
% %         Kin.setStatesStore(StoData);
% %         Kin.set_statesStore(StoData);
% %         Kin.setStepInterval(0.001);
% % 
% %         Kin.setStartTime(initial_time);
% %         Kin.setEndTime(final_time);
% %         Kin.setInDegrees(true);
% % 
% %         model.addAnalysis(Kin)
% % 
% % 
% % %Tool
% % analyzeTool.setModel(model);
% % analyzeTool.setName(name);
% % analyzeTool.setResultsDir(results_folder);
% % % analyzeTool.setSoveForEquilibrium(false);%
% % analyzeTool.setStatesFileName("D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M - OS40\AnalizeResults\Resultados_MuscleAnalysis_FiberForce_Spastic.sto");
% % % analyzeTool.setCoordinatesFileName(motIKCoordsFile);
% % analyzeTool.setStatesStorage(StoData);
% % analyzeTool.setInitialTime(initial_time);
% % analyzeTool.setFinalTime(final_time);   
% % analyzeTool.setPrintResultFiles(true);
% % analyzeTool.setLoadModelAndInput(true);
% % outfile = ['Setup_Analyze_' name '.xml'];
% % analyzeTool.print(fullfile(CD_model, outfile));
% % 
% %     %Ejecuto solo un análisis cinematico
% %     MuscleAnalysis=model.getAnalysisSet.get(2);
% %     model.removeAnalysis(MuscleAnalysis);
% % %     MuscleAnalysis=model.getAnalysisSet.get(0)
% % %     model.removeAnalysis(MuscleAnalysis);
% %     
% %     if model.getNumAnalyses == 1
% % analyzeTool.run();
% %     end
% % disp(strcat('Analisis realizado correctamente y guardado en: ', results_folder));
% % disp(strcat(' XML del análisis -> ',outfile));
% 
% %% NUEVA FD
% if ELECCION == 3
% 
% %     INPUTS:
%     MAsto=org.opensim.modeling.Storage(New_NFV_Path);
%     
% % Tiempos:
%     StartTime=MAsto.getFirstTime;
%     LastTime=MAsto.getLastTime;    
%     
%     
% %   Analisis:
%     Kin=Kinematics();
%     Kin.set_model(model);
%     Kin.set_statesStore(MAsto);
% %     Kin.setStepInterval(0.001);%?
%     Kin.setStartTime(StartTime);
%     Kin.setEndTime(LastTime);
% %     Kin.setInDegrees(true);
%         model.addAnalysis(Kin)
% 
%     
% % Initialize the underlying computational system
% state = model.initSystem();
% 
% % Create the Forward Tool
% tool = ForwardTool();
% 
% % Set the model for the forward tool
% tool.setModel(model);
% 
% % Define the start and finish times 
% tool.setStartTime(StartTime);
% tool.setFinalTime(LastTime);
% 
% % Define the prefix for the result files
% tool.setName('NewFD');
% 
% % Set Input States File
% % tool.setStatesFileName("D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M - OS40\AnalizeResults\AnalyzeResults_MuscleAnalysis_FiberForce.sto");
% tool.setStatesFileName(New_NFV_Path);
% % Set Results Directory (will create without prompt)
% tool.setResultsDir('D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M - OS40\FD_Output');
% 
% % Run the simulation
% statusVal = tool.run();
% 
% FDOutputPath='D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M - OS40\FD_Output\NewFD_states_degrees.mot';
% 
% % Cleanup
% % clearvars walkerModel forceReporter tool state statusVal
% display('Forward Tool Finished.');
% 
% end
% 
% 
% 
% %% DINAMICA DIRECTA FD
% if ELECCION == 1
% [FDOutputPath] = f_FD(model,New_NFVelocity_Storage,New_NFV_Path,CD_model);
% pause(0.2);
% 
% end
% %% DINAMICA DIRECTA FD (MODELO DE FUERZA)
% if ELECCION == 2
%     [FDOutputPath] = f_FD(model,New_FForce_Storage,New_FF_Path,CD_model);
%     pause(0.2);
% end
% % Devuelve las velocidades y valores de las COORDENADAS ARTICULARES
% % (_states.sto)
% % es posible que si cojo solo los valores sea como Movimiento.mot pero ya
% % modificado, y me deje trabajar con el.
% %% Adapto la salida de FD a un motion cogiendo solo los valores y no las velocidades
% 
% %   2 opciones: 1- _states.sto -> ya esta en radianes y creo que se puede
% %   trabajar con el .sto igual que con el .mot
% %               2- states_degrees.mot -> habría que pasarlo a radianes pero
% %               ya es un .mot
% 
% % OPCIÓN 1:
% %Copio los datos de _states.sto
% %Copio la estructura del archivo de Movimiento.mot
% 
% VelAndValuesSTO=org.opensim.modeling.Storage(strcat(FDOutputPath,'FDOutput_states.sto'));
% 
% % DATOS
%     VelAndValues=importdata(strcat(FDOutputPath,'FDOutput_states.sto'));
%         Values=VelAndValues.data(:,2:2:40);
%         Time=VelAndValues.data(:,1);
%             TimeAndValues=[Time,Values];
% % ESTRUCTURA
%     OldSto=org.opensim.modeling.Storage(strcat(CD_CArticulares,"\Movimiento.mot"));
%     %Lo vacio
%     OldSto.reset(int16(0));
%     % Modifico la descripcion
%     OldSto.setInDegrees(true);
% 
%     
%     %Meto los valores COMO STATEVECTORS
%     for i=1:length(TimeAndValues(:,1))
%         fila=org.opensim.modeling.StateVector();
%         v=org.opensim.modeling.Vector();
%         v.resize(int16(length(Values(1,:))));
%         for j = 1:length(Values(1,:))
%                 v.set(int16(j-1),Values(i,j)); %Vector empieza desde 0
%             fila.setStates(1,v);
%             fila.setTime(Time(i));
%             size=fila.getSize;
%         end
%         OldSto.append(fila);
%     end
%     
%     
% OldSto.print(strcat(FDOutputPath,'FDOutput_ValuesFDSto.sto'));
% 
% % OPCIÓN 2:
% %Copio la estructura y los datos del archivo de Movimiento.mot y lo dejo en
% %grados
% 
% % VelAndValuesSTO=org.opensim.modeling.Storage(strcat(FDOutputPath,'\_states_degrees.mot'));
% 
% % DATOS
%     VelAndValues=importdata(strcat(FDOutputPath,'FDOutput_states_degrees.mot'));
%         Values=VelAndValues.data(:,2:2:40);
%         Time=VelAndValues.data(:,1);
%             TimeAndValues=[Time,Values];
% % ESTRUCTURA
%     OldMot=org.opensim.modeling.Storage(strcat(CD_CArticulares,"\Movimiento.mot"));
%         CL=OldMot.getColumnLabels;
%     NewMot=org.opensim.modeling.Storage(strcat(FDOutputPath,'FDOutput_states_degrees.mot'));
%     %Lo vacio
%     NewMot.reset(int16(0));
%         NewMot.setColumnLabels(CL);
%     % Modifico la descripcion
% %     OldSto.setInDegrees(true);
% 
%     
%     %Meto los valores COMO STATEVECTORS
%     for i=1:length(TimeAndValues(:,1))
%         fila=org.opensim.modeling.StateVector();
%         v=org.opensim.modeling.Vector();
%         v.resize(int16(length(Values(1,:))));
%         for j = 1:length(Values(1,:))
%                 v.set(int16(j-1),Values(i,j)); %Vector empieza desde 0
%             fila.setStates(1,v);
%             fila.setTime(Time(i));
%             size=fila.getSize;
%         end
%         NewMot.append(fila);
%     end
%     
%     
% NewMot.print(strcat(FDOutputPath,'FDOutput_ValuesFDMot.mot'));
% 
% disp('ACABADO');


% ----------------------------------------------------
% REPRODUZCO EL MOVIMIENTO MODIFICADO
% % CD_FDOutput='D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M - OS40\FD_Output';
% Motion="FDOutput_ValuesFDMot.mot";
% % Motion="FDOutput_ValuesFDSto.sto";
% f_PlayVisualizer(FDOutputPath,model,Motion)
% 
% 
% 
% %  f_PlayVisualizer("D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M - OS40\PRUEBAS\",model,"prueba4_states_degrees.mot")
% 
% 
% disp('REPRODUCCIÓN ACABADA');