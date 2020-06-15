function [TrcTableCreada] = f_CreateTRCTable(t,CMarkers,dataUsed)
% Paso los datos de las coordenadas almacenados en CMarkers a un archivo
% TRC para poder leerlo desde OpenSim.
if (isequal(dataUsed ,'IIWA_Kinect') ||isequal(dataUsed ,'Kinect'))
    numMarkers='14';
    NFrames=num2str(length(CMarkers.HandRight(:,1)));%QUITAR!!!!!!!!!!!!!!!
else
    numMarkers='1';
    NFrames=num2str(length(CMarkers.Handle(:,1))); 

end
TrcTableCreada=org.opensim.modeling.TimeSeriesTableVec3();
TrcTableCreada.addTableMetaDataString('DataRate','60');
TrcTableCreada.addTableMetaDataString('CameraRate','60');
TrcTableCreada.addTableMetaDataString('NumFrames',NFrames);
TrcTableCreada.addTableMetaDataString('NumMarkers',numMarkers);
TrcTableCreada.addTableMetaDataString('Units','m');  %!!!
TrcTableCreada.addTableMetaDataString('OrigDataRate','60');
TrcTableCreada.addTableMetaDataString('OrigDataStart','1');
TrcTableCreada.addTableMetaDataString('OrigNumFrames',NFrames);

%Añado una fila y una columna inicial:
if (strcmp(org.opensim.modeling.opensimCommon.GetVersion(), '4.1'))
    RV=org.opensim.modeling.RowVectorVec3();
else
    RV=org.opensim.modeling.RowVectorOfVec3();
end
TrcTableCreada.appendRow(1,RV); %añado una fila vacia

%Añado la columna:
%Labels:
MarkersNames = org.opensim.modeling.StdVectorString();
TrcTableCreada.setColumnLabels(MarkersNames);
V=org.opensim.modeling.Vec3(1,1,1);
if (strcmp(org.opensim.modeling.opensimCommon.GetVersion(), '4.1'))
    VV=org.opensim.modeling.VectorVec3(1,V);
else
    VV=org.opensim.modeling.VectorOfVec3(1,V);
end
if isequal(dataUsed ,'IIWA')
    TrcTableCreada.appendColumn('Handle',VV);
    dataLength=length(CMarkers.Handle(:,1)); % Revisar cuando se tengan todos los datos
end
if (isequal(dataUsed ,'IIWA_Kinect') ||isequal(dataUsed ,'Kinect'))
    TrcTableCreada.appendColumn('Handle',VV);
    TrcTableCreada.appendColumn('SpineBase',VV);
    TrcTableCreada.appendColumn('SpineMid',VV);
    TrcTableCreada.appendColumn('Neck',VV);
    TrcTableCreada.appendColumn('Head',VV);
    TrcTableCreada.appendColumn('ShoulderRight',VV);
    TrcTableCreada.appendColumn('ElbowRight',VV);
    TrcTableCreada.appendColumn('WristRight',VV);
    TrcTableCreada.appendColumn('HandRight',VV);
    TrcTableCreada.appendColumn('HipLeft',VV);
    TrcTableCreada.appendColumn('HipRight',VV);
    TrcTableCreada.appendColumn('SpineShoulder',VV);
    TrcTableCreada.appendColumn('HandTipRight',VV);
    TrcTableCreada.appendColumn('ThumbRight',VV);
    dataLength=length(CMarkers.SpineBase(:,1));
end

TrcTableCreada.removeRowAtIndex(0);
%hasta aquí la tabla está vacia(solo tiene head y labels)
%Pruebo a meterle directamente las filas buenas, en vez de
%meter una al azar y luego modificarla


%Añado el numero de filas de la tabla del laboratorio

for i= 1:dataLength %BUCLE DE ROWS
    if (strcmp(org.opensim.modeling.opensimCommon.GetVersion(), '4.1'))
        RV=org.opensim.modeling.RowVectorVec3();
    else
        RV=org.opensim.modeling.RowVectorOfVec3();
    end
    if (isequal(dataUsed ,'IIWA_Kinect') ||isequal(dataUsed ,'Kinect'))
        RV.resize(14);
        
        if isequal(dataUsed ,'Kinect')
            %Handle = HandRight
            V=org.opensim.modeling.Vec3(CMarkers.HandRight(i,1),CMarkers.HandRight(i,2),CMarkers.HandRight(i,3));
            RV.set(0,V);
        else %Si se está utilizando el IIWA
            %Handle = Trayectoria TCP
            V=org.opensim.modeling.Vec3(CMarkers.Handle(i,1),CMarkers.Handle(i,2),CMarkers.Handle(i,3));
            RV.set(0,V)
        end
        %SpineBase
        VSpineBase=org.opensim.modeling.Vec3(CMarkers.SpineBase(i,1),CMarkers.SpineBase(i,2),CMarkers.SpineBase(i,3));
        RV.set(1,VSpineBase); %En la segunda columna
        %SpineMid
        VSpineMid=org.opensim.modeling.Vec3(CMarkers.SpineMid(i,1),CMarkers.SpineMid(i,2),CMarkers.SpineMid(i,3));
        RV.set(2,VSpineMid); %En la tercera columna
        %Neck
        VNeck=org.opensim.modeling.Vec3(CMarkers.Neck(i,1),CMarkers.Neck(i,2),CMarkers.Neck(i,3));
        RV.set(3,VNeck); %En la cuarta columna
        %Head
        VHead=org.opensim.modeling.Vec3(CMarkers.Head(i,1),CMarkers.Head(i,2),CMarkers.Head(i,3));
        RV.set(4,VHead); %En la quinta columna
        %ShoulderRight
        VShoulderRight=org.opensim.modeling.Vec3(CMarkers.ShoulderRight(i,1),CMarkers.ShoulderRight(i,2),CMarkers.ShoulderRight(i,3));
        RV.set(5,VShoulderRight);
        %ElbowRight
        VElbowRight=org.opensim.modeling.Vec3(CMarkers.ElbowRight(i,1),CMarkers.ElbowRight(i,2),CMarkers.ElbowRight(i,3));
        RV.set(6,VElbowRight);
        %WristRight
        VWristRight=org.opensim.modeling.Vec3(CMarkers.WristRight(i,1),CMarkers.WristRight(i,2),CMarkers.WristRight(i,3));
        RV.set(7,VWristRight);
        %HandRight
        VHandRight=org.opensim.modeling.Vec3(CMarkers.HandRight(i,1),CMarkers.HandRight(i,2),CMarkers.HandRight(i,3));
        RV.set(8,VHandRight);
        %HipLeft
        VHipLeft=org.opensim.modeling.Vec3(CMarkers.HipLeft(i,1),CMarkers.HipLeft(i,2),CMarkers.HipLeft(i,3));
        RV.set(9,VHipLeft);
        %HipRight
        VHipRight=org.opensim.modeling.Vec3(CMarkers.HipRight(i,1),CMarkers.HipRight(i,2),CMarkers.HipRight(i,3));
        RV.set(10,VHipRight);
        %SpineShoulder
        VSpineShoulder=org.opensim.modeling.Vec3(CMarkers.SpineShoulder(i,1),CMarkers.SpineShoulder(i,2),CMarkers.SpineShoulder(i,3));
        RV.set(11,VSpineShoulder);
        %ThumbRight
        VThumbRight=org.opensim.modeling.Vec3(CMarkers.ThumbRight(i,1),CMarkers.ThumbRight(i,2),CMarkers.ThumbRight(i,3));
        RV.set(12,VThumbRight);
        %HandTipRight
        VHandTipRight=org.opensim.modeling.Vec3(CMarkers.HandTipRight(i,1),CMarkers.HandTipRight(i,2),CMarkers.HandTipRight(i,3));
        RV.set(13,VHandTipRight);
    end
    if isequal(dataUsed ,'IIWA')
        RV.resize(1);
        %Handle = Trayectoria TCP
        V=org.opensim.modeling.Vec3(CMarkers.Handle(i,1),CMarkers.Handle(i,2),CMarkers.Handle(i,3));
        RV.set(0,V)
    end
    TrcTableCreada.appendRow(i,RV)
    TrcTableCreada.setIndependentValueAtIndex(i-1,t(i));
    i;
end
% OUTPUT
TrcTableCreada;
end


