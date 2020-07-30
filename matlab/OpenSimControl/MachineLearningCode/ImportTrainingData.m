
%%%%%%%%%%%%%%MODIFICABLES%%%%%%%%%%%%%%%%%%%%
% EL PROGRAMA ESTA PENSADO PARA USAR EN ESTA DEFINICION, 11 FUERZAS
% DISTINTAS POR CADA GANANCIA, POR LO QUE POR CADA TRAY DE KINECT SE
% RECOLECTARAN 11X6 : con el siguiente LOOP  KINECT(GANACIAS(IIWA))

%Por lo tanto para esta definicion descrita el algoritmo cuenta con dos
%carpetas de Tray de Kinect (6 y 7), en cada una de ellas seis carpetas con
%las ganacias  de Ashworth, y a su vez en cada una de ellas 11 carpetas de
%las diferentes fuerzas aplicadas

available_trays=2; % Numero de trays de Kinect utilizadas
KinectTrays = [6,7]; %Trays de Kinect utilizadas
gAshworth = 6; %numero de ganancias en la tabla Ashworth
iiwatrays = 10; %numero de trays del iiwa a una ganancia
ConvertDataFcy = 0; %Usar datos en frecuencia instantanea para el entrenamiento
%Modificar nombre de los archivos obtenidos desde Opensim despues del FD
FDnameFile = 'MoBL_Flexion_SpasticMillard-scaled_';


file_states = [FDnameFile 'states.sto'];
file_control = [FDnameFile 'controls.sto'];

%Otras posibles features que se le pueden agregar al conjunto de datos de
%entrada, al agregar mas el algoritmo es mas propenso a overfitting
%En caso de usar modificar nombre, MuscleAnalysis
file_vel = [FDnameFile 'MuscleAnalysis_' 'NormFiberVelocity.sto'];
file_force = [FDnameFile 'MuscleAnalysis_' 'PassiveFiberForce.sto'];
file_ActFiForce = [FDnameFile 'MuscleAnalysis_' 'ActiveFiberForce.sto'];
file_ActFib_tendon = [FDnameFile 'MuscleAnalysis_' 'ActiveFiberForceAlongTendon.sto'];
file_FiberActPow = [FDnameFile 'MuscleAnalysis_' 'FiberActivePower.sto'];
file_Fiberforce = [FDnameFile 'MuscleAnalysis_' 'FiberForce.sto'];
file_fiberpasForce = [FDnameFile 'MuscleAnalysis_' 'FiberPassivePower.sto'];
file_lenght = [FDnameFile 'MuscleAnalysis_' 'Length.sto'];
file_muscleAcPower = [FDnameFile 'MuscleAnalysis_' 'MuscleActuatorPower.sto'];
file_normFiber = [FDnameFile 'MuscleAnalysis_' 'NormalizedFiberLength.sto'];
file_passFibFor = [FDnameFile 'MuscleAnalysis_' 'PassiveFiberForce.sto'];
file_passFibTendon = [FDnameFile 'MuscleAnalysis_' 'PassiveFiberForceAlongTendon.sto'];
file_PennationAngle = [FDnameFile 'MuscleAnalysis_' 'PennationAngle.sto'];
file_pennationVel = [FDnameFile 'MuscleAnalysis_' 'PennationAngularVelocity.sto'];
file_tendonForce = [FDnameFile 'MuscleAnalysis_' 'TendonForce.sto'];
file_tendonLength = [FDnameFile 'MuscleAnalysis_' 'TendonLength.sto'];
file_tendonpower = [FDnameFile 'MuscleAnalysis_' 'TendonPower.sto'];


pathTrainingData=[];
title=strcat('Selecciona el directorio del conjunto de datos') ;
 pathTrainingData = uigetdir(pathTrainingData, title);
 if (pathTrainingData == 0)
     ME = MException('Main:NopathTrainingData', 'No training path selected');
    throw(ME);
 end
 disp(['Using training data path: ', pathTrainingData]);
 
 %Tarda un poco
 L=1;
for i=1:available_trays
for j=1:gAshworth

for k=0:iiwatrays    
    
cd_FD = strcat(pathTrainingData,'\Tray', string(KinectTrays(i)), '\G',g_labels{j},'\TrayIIWA',string(k));
FD_states = importfile1( strcat(cd_FD ,'\',file_states));

%Descomentar imports que se quieren hacer

%FD_vel = importfile1( strcat( cd_FD,'\', file_vel));
%FD_force = importfile1( strcat(cd_FD,'\',file_force));
%FD_controls = importfile1( strcat(cd_FD ,'\',file_control));
%FD_ActfiForce = importfile1( strcat(cd_FD ,'\',file_ActFiForce));
%FD_ActFib_tendon = importfile1( strcat(cd_FD ,'\',file_ActFib_tendon));
%FD_FiberActPow = importfile1( strcat(cd_FD ,'\',file_FiberActPow));
%FD_Fiberforce = importfile1( strcat(cd_FD ,'\',file_Fiberforce));
%FD_fiberpasForce = importfile1( strcat(cd_FD ,'\',file_fiberpasForce));
%FD_lenght = importfile1( strcat(cd_FD ,'\',file_lenght));
%FD_muscleAcPower = importfile1( strcat(cd_FD ,'\',file_muscleAcPower));
%FD_normFiber = importfile1( strcat(cd_FD ,'\',file_normFiber));
%FD_passFibFor= importfile1( strcat(cd_FD ,'\',file_passFibFor));
%FD_passFibTendon = importfile1( strcat(cd_FD ,'\',file_passFibTendon));
%FD_PennationAngle = importfile1( strcat(cd_FD ,'\',file_PennationAngle));
%FD_pennationVel = importfile1( strcat(cd_FD ,'\',file_pennationVel));
%FD_tendonForce = importfile1( strcat(cd_FD ,'\',file_tendonForce));
%FD_tendoLength = importfile1( strcat(cd_FD ,'\',file_tendonLength));
%FD_tendopower = importfile1( strcat(cd_FD ,'\',file_tendonpower));


x_train{L} = [ FD_states.x_forceset_BIClong_activation';  FD_states.x_forceset_BICshort_activation'; FD_states.x_forceset_BRA_activation'];

if ConvertDataFcy == 1
    %Si queremos ver el comportamiento de los datos en frecuencia
    %instantanea por ej, para la conversion, la estructura de datos deber
    %ser diferente.
    x_train{L} = [ FD_states.x_forceset_BIClong_activation,  FD_states.x_forceset_BICshort_activation, FD_states.x_forceset_BRA_activation];

end

%{
         TODAS LAS VARIABLES DEL ANALISIS MUSCULAR         
               
x_train{L} = [ FD_states.x_forceset_BIClong_activation';  FD_states.x_forceset_BICshort_activation'; FD_states.x_forceset_BRA_activation';...
               FD_controls.BICshort' ; FD_controls.BIClong'; FD_controls.BRA';...
               FD_vel.BIClong'; FD_vel.BICshort'; FD_force.BIClong'; FD_force.BICshort';...
               FD_ActFib_tendon.BIClong';FD_ActFib_tendon.BICshort';FD_ActFib_tendon.BRA';...
               FD_FiberActPow.BIClong';FD_FiberActPow.BICshort';FD_FiberActPow.BRA';...
               FD_Fiberforce.BIClong';FD_Fiberforce.BICshort';FD_Fiberforce.BRA';...
               FD_fiberpasForce.BIClong';FD_fiberpasForce.BICshort';FD_fiberpasForce.BRA';...
               FD_lenght.BIClong';FD_lenght.BICshort';FD_lenght.BRA';...
               FD_muscleAcPower.BIClong';FD_muscleAcPower.BICshort';FD_muscleAcPower.BRA';...
               FD_normFiber.BIClong';FD_normFiber.BICshort';FD_normFiber.BRA';...
               FD_passFibFor.BIClong';FD_passFibFor.BICshort';FD_passFibFor.BRA';...
               FD_passFibTendon.BIClong';FD_passFibTendon.BICshort';FD_passFibTendon.BRA';...
               FD_PennationAngle.BIClong';FD_PennationAngle.BICshort';FD_PennationAngle.BRA';...
               FD_pennationVel.BIClong';FD_pennationVel.BICshort';FD_pennationVel.BRA';...
               FD_tendonForce.BIClong';FD_tendonForce.BICshort';...
               FD_tendoLength.BIClong';FD_tendoLength.BICshort';FD_tendoLength.BRA';...
               FD_tendopower.BIClong';FD_tendopower.BICshort';FD_tendopower.BRA'];
%}



y_train{L} = g_labels{j};
L=L+1;
end
end
end

%{
%EN UN CASO FUTURO DONDE NO SE TENGA KINECT, NI SE VARIEN LAS FUERZAS DEL
%IIWA DE UN MISMO INDIVIDUO; SE TENDRIA SOLO GANANCIAS, POR LO QUE SE
%NECESITARIA SOLO CARPETAS DE LAS GANANCIAS

%Mi idea o recomendacion seria hacer carpetas que contengan todos los
analisis musculares por persona, dentro de diferentes ganancias **
                
            G1.1 { Persona1 { states, controls etc ... }

                   Persona2
                        ....}

 L=1;

for j=1:gAshworth

for k=0:NUMERODEPERSONAS**    
    
cd_FD = strcat(pathTainingData,'\G',g_labels{j},'\Persona',string(k));
FD_states = importfile1( strcat(cd_FD ,'\',file_states));


x_train {L} = [ FD_states.x_forceset_BIClong_activation';  FD_states.x_forceset_BICshort_activation'; FD_states.x_forceset_BRA_activation'];

y_train{L} = g_labels{j};
L=L+1;
end
end
end

%}


X = x_train;
Labels = categorical(y_train);