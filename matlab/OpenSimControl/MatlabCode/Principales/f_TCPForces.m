function [ForceAndTorque,stamps] = f_TCPForces(Datos,DatosVacio,tsample)

torquesVacio=DatosVacio{1}.fuerza;
stampsVacio=DatosVacio{1}.stamps_fuerza;
    [stamps, torquesVacio]=equidistant(stampsVacio, torquesVacio, tsample);

torquesCargado=Datos{1}.fuerza;
torqueStampsCargado=Datos{1}.stamps_fuerza;
    [~, torquesCargado]=equidistant(torqueStampsCargado, torquesCargado, tsample);
    
jp_cargado=Datos{1}.trayectoria;
jp_stampsCargado=Datos{1}.stamps;
    [~, jp_cargado]=equidistant(jp_stampsCargado, jp_cargado, tsample);

cp_cargado = FK(jp_cargado);

robot = importrobot('./IIWA/iiwa14.urdf');
configuration = randomConfiguration(robot);

Ytr=spline(1:size(jp_cargado,2), jp_cargado, linspace(1,size(jp_cargado,2),size(torquesCargado,2)));

F1=[];
F2=[];

for n=1:size(torquesCargado,2)
    
    for i=1:7
        configuration(i).JointPosition=Ytr(i,n);
    end
    
    J=geometricJacobian(robot,configuration,'iiwa_link_ee');
    
    pseudoinvJ1=inv(J*J')*J;  %En matlab se traspone al reves
    %pseudoinvJ2=(J*J')\J;
    diff_torques=torquesCargado(:,i)-torquesVacio(:,i);

%     F1(:,n)=pseudoinvJ1*diff_torques;
    F2(:,n)=pinv(J')*diff_torques;

%     %resto las cartesianas y sale exactamente lo mismo
%     F3(:,n)=pseudoinvJ1*torquesCargado(:,i)-pseudoinvJ1*torquesVacio(:,i);
%     F4(:,n)=pinv(J')*torquesCargado(:,i) -pinv(J')*torquesVacio(:,i);
%     
%     % dibujo las totales sin resta
%     F5(:,n)=pinv(J')*torquesCargado(:,i);
%     F6(:,n)=pinv(J')*torquesVacio(:,i);
    
    
end

%ForceAndTorque=F1';
% Por lo que sea las fuerzas salen ya invertidas, por lo que se las pasa
% directamente a la mano
ForceAndTorque(:,1:3)=F2(1:3,:)';
ForceAndTorque(:,7:9)=F2(4:6,:)';
%Añado el point donde se aplica la fuerza (ACTUALMENTE NO SE UTILIZA PORQUE
%SE DECIDIÓ UTILIZAR UNA DISTANCIA FIJA AL SISTEMA DE  COORD DE LA MANO
%(MIRAR MEMORIA TFM EDUARDO SÁENZ)
% ForceAndTorque(:,4)=CMarkers.Handle.x';
% ForceAndTorque(:,5)=CMarkers.Handle.y';
% ForceAndTorque(:,6)=CMarkers.Handle.z';

% EN EL CASO DE QUERER REPRESENTAR LAS FUERZAS:
% figure(1);
% hold on;
% axis equal;
% ft=ForceAndTorque(:,1:3)./50;
% quiver3(cp_cargado(1,:), cp_cargado(2,:), cp_cargado(3,:), ft(:,1)', ft(:,2)', ft(:,3)', 0, 'b');
% % legend('sin', 'con');

% figure(1);
%  hold on;
%  axis equal;
%  ft=F5(1:3,:)'/500;
%  fy=ft(:,2);
%  ft(:,2)=-ft(:,3);
%  ft(:,3)=fy;
%  quiver3(cp_cargado(1,:), cp_cargado(2,:), cp_cargado(3,:), ft(:,1)', ft(:,2)', ft(:,3)', 0, 'b');
end



