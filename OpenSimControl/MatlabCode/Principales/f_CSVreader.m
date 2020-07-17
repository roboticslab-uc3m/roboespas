function CMarkers = f_CSVreader(filepath,filename,startRow,endRow)
% READ CSV
cd(filepath);
% filename=fullfile('TrayEdu1.csv');

% fileID = fopen(filename);
% N=75; %Columnas a leer
% formatSpec = '%q';
% i=2;
% Markers = textscan(fileID,formatSpec,N,'Delimiter',';','EmptyValue',0);
% Head = textscan(fileID,formatSpec,N,'Delimiter',';','EmptyValue',0);

%los valores los leo con la función import data
Valores = f_importCSV(filename, startRow, endRow); 
numRows=length(Valores(:,1));



%corregimos su orientación, tomando como referencia los markers de
%spineshoulder y rightshoulder en el modelo de OpenSim
%spineShoulder/ground (-0.000945343 -0.0143921 -0.00059369)
%rightShoulder/ground (-0.0250445 0.00412053 0.156821)
%si los alineamos en x:
offsetX=-0.0250445-(-0.000945343); %Estos valores son CONSTANTES y pertenecen al modelo de OpenSim

%     ShoulderRight_Z - SpineShoulder_Z + offsetX   (ZKinect = X)
difX=-1*(Valores(3,27)- Valores(3,63))+offsetX; % considero que al ser offsetX tan pequeño no es necesario girarlo

%     ShoulderRight_X - SpineShoulder_X   (XKinect = Z)
difZ=Valores(3,25)-Valores(3,61); %considero el offsetZ=0

    alpha=atan(difX/difZ); %en radianes

%Matriz de Rotación sobre y
      Ry=[cos(alpha) 0 sin(alpha);0 1 0;-sin(alpha) 0 cos(alpha)];
      
%LLevo el origen del Sist de Coordenadas al punto SpineShoulder y hago la
%rotación:
SpineBase(:,3) = Valores(3:numRows,1)-Valores(3,61); 
SpineBase(:,2) = Valores(3:numRows,2)-Valores(3,62)-0.03; 
SpineBase(:,1) = -1*(Valores(3:numRows,3)-Valores(3,63)); 
CMarkers.SpineBase=SpineBase*Ry;

SpineMid(:,3) = Valores(3:numRows,4)-Valores(3,61); 
SpineMid(:,2) = Valores(3:numRows,5)-Valores(3,62)-0.03; 
SpineMid(:,1) = -1*(Valores(3:numRows,6)-Valores(3,63)); 
CMarkers.SpineMid=SpineMid*Ry;

Neck(:,3) = Valores(3:numRows,7)-Valores(3,61); 
Neck(:,2) = Valores(3:numRows,8)-Valores(3,62)-0.03; 
Neck(:,1) = -1*(Valores(3:numRows,9)-Valores(3,63)); 
CMarkers.Neck=Neck*Ry;

Head(:,3) = Valores(3:numRows,10)-Valores(3,61); 
Head(:,2) = Valores(3:numRows,11)-Valores(3,62)-0.03; 
Head(:,1) = -1*(Valores(3:numRows,12)-Valores(3,63)); 
CMarkers.Head=Head*Ry;

ShoulderRight(:,3) = Valores(3:numRows,25)-Valores(3,61); 
ShoulderRight(:,2) = Valores(3:numRows,26)-Valores(3,62)-0.03; 
ShoulderRight(:,1) = -1*(Valores(3:numRows,27)-Valores(3,63)); 
CMarkers.ShoulderRight=ShoulderRight*Ry;

ElbowRight(:,3) = Valores(3:numRows,28)-Valores(3,61); 
ElbowRight(:,2) = Valores(3:numRows,29)-Valores(3,62)-0.03; 
ElbowRight(:,1) = -1*(Valores(3:numRows,30)-Valores(3,63));
CMarkers.ElbowRight=ElbowRight*Ry;

WristRight(:,3) = Valores(3:numRows,31)-Valores(3,61); 
WristRight(:,2) = Valores(3:numRows,32)-Valores(3,62)-0.03; 
WristRight(:,1) = -1*(Valores(3:numRows,33)-Valores(3,63)); 
CMarkers.WristRight=WristRight*Ry;

HandRight(:,3) = Valores(3:numRows,34)-Valores(3,61); 
HandRight(:,2) = Valores(3:numRows,35)-Valores(3,62)-0.03; 
HandRight(:,1) = -1*(Valores(3:numRows,36)-Valores(3,63)); 
CMarkers.HandRight=HandRight*Ry;

HipLeft(:,3) = Valores(3:numRows,37)-Valores(3,61); 
HipLeft(:,2) = Valores(3:numRows,38)-Valores(3,62)-0.03; 
HipLeft(:,1) = -1*(Valores(3:numRows,39)-Valores(3,63)); 
CMarkers.HipLeft=HipLeft*Ry;

HipRight(:,3) = Valores(3:numRows,49)-Valores(3,61); 
HipRight(:,2) = Valores(3:numRows,50)-Valores(3,62)-0.03; 
HipRight(:,1) = -1*(Valores(3:numRows,51)-Valores(3,63)); 
CMarkers.HipRight=HipRight*Ry;

SpineShoulder(:,3) = Valores(3:numRows,61)-Valores(3,61); 
SpineShoulder(:,2) = Valores(3:numRows,62)-Valores(3,62)-0.03; 
SpineShoulder(:,1) = -1*(Valores(3:numRows,63)-Valores(3,63)); 
CMarkers.SpineShoulder=SpineShoulder*Ry;

HandTipRight(:,3) = Valores(3:numRows,70)-Valores(3,61); 
HandTipRight(:,2) = Valores(3:numRows,71)-Valores(3,62)-0.03; 
HandTipRight(:,1) = -1*(Valores(3:numRows,72)-Valores(3,63)); 
CMarkers.HandTipRight=HandTipRight*Ry;

ThumbRight(:,3) = Valores(3:numRows,73)-Valores(3,61); 
ThumbRight(:,2) = Valores(3:numRows,74)-Valores(3,62)-0.03; 
ThumbRight(:,1) = -1*(Valores(3:numRows,75)-Valores(3,63)); 
CMarkers.ThumbRight=ThumbRight*Ry;


% CMarkers.SpineMid.z = Valores(3:numRows,4)-Valores(3,61); 
% CMarkers.SpineMid.y = Valores(3:numRows,5)-Valores(3,62); 
% CMarkers.SpineMid.x = -1*(Valores(3:numRows,6)-Valores(3,63)); 
% 
% CMarkers.Neck.z = Valores(3:numRows,7)-Valores(3,61); 
% CMarkers.Neck.y = Valores(3:numRows,8)-Valores(3,62); 
% CMarkers.Neck.x = -1*(Valores(3:numRows,9)-Valores(3,63)); 
% 
% CMarkers.Head.z = Valores(3:numRows,10)-Valores(3,61); 
% CMarkers.Head.y = Valores(3:numRows,11)-Valores(3,62); 
% CMarkers.Head.x = -1*(Valores(3:numRows,12)-Valores(3,63)); 
% 
% CMarkers.ShoulderRight.z = Valores(3:numRows,25)-Valores(3,61); 
% CMarkers.ShoulderRight.y = Valores(3:numRows,26)-Valores(3,62); 
% CMarkers.ShoulderRight.x = -1*(Valores(3:numRows,27)-Valores(3,63)); 
% 
% CMarkers.ElbowRight.z = Valores(3:numRows,28)-Valores(3,61); 
% CMarkers.ElbowRight.y = Valores(3:numRows,29)-Valores(3,62); 
% CMarkers.ElbowRight.x = -1*(Valores(3:numRows,30)-Valores(3,63));
% 
% CMarkers.WristRight.z = Valores(3:numRows,31)-Valores(3,61); 
% CMarkers.WristRight.y = Valores(3:numRows,32)-Valores(3,62); 
% CMarkers.WristRight.x = -1*(Valores(3:numRows,33)-Valores(3,63)); 
% 
% CMarkers.HandRight.z = Valores(3:numRows,34)-Valores(3,61); 
% CMarkers.HandRight.y = Valores(3:numRows,35)-Valores(3,62); 
% CMarkers.HandRight.x = -1*(Valores(3:numRows,36)-Valores(3,63)); 
% 
% CMarkers.HipLeft.z = Valores(3:numRows,37)-Valores(3,61); 
% CMarkers.HipLeft.y = Valores(3:numRows,38)-Valores(3,62); 
% CMarkers.HipLeft.x = -1*(Valores(3:numRows,39)-Valores(3,63)); 
% 
% CMarkers.HipRight.z = Valores(3:numRows,49)-Valores(3,61); 
% CMarkers.HipRight.y = Valores(3:numRows,50)-Valores(3,62); 
% CMarkers.HipRight.x = -1*(Valores(3:numRows,51)-Valores(3,63)); 
% 
% CMarkers.SpineShoulder.z = Valores(3:numRows,61)-Valores(3,61); 
% CMarkers.SpineShoulder.y = Valores(3:numRows,62)-Valores(3,62); 
% CMarkers.SpineShoulder.x = -1*(Valores(3:numRows,63)-Valores(3,63)); 
% 
% CMarkers.HandTipRight.z = Valores(3:numRows,70)-Valores(3,61); 
% CMarkers.HandTipRight.y = Valores(3:numRows,71)-Valores(3,62); 
% CMarkers.HandTipRight.x = -1*(Valores(3:numRows,72)-Valores(3,63)); 
% 
% CMarkers.ThumbRight.z = Valores(3:numRows,73)-Valores(3,61); 
% CMarkers.ThumbRight.y = Valores(3:numRows,74)-Valores(3,62); 
% CMarkers.ThumbRight.x = -1*(Valores(3:numRows,75)-Valores(3,63)); 



% Valores(1,:) = textscan(fileID,formatSpec,N,'Delimiter',';','EmptyValue',0);
% while Valores{i-1,1}{1,1} %~= "[]"
%         t=textscan(fileID,formatSpec,N,'Delimiter',';','EmptyValue',0);
%         Valores{i,1} = t{1,1};
%         i=i+1;
% end 
%Data = str2double(strrep(Datos{2,1}{4,1}, ',', '.'));


end