% Fuerzas en el TCP

cd('D:\User\escrit\Universidad\Master\2\TFM\Codigos Mar\MyCode\MyCode');
robot = importrobot('iiwa14.urdf');
configuration = randomConfiguration(robot);

cd('D:\User\escrit\Universidad\Master\2\TFM\Movimientos\Codo sagital\codo_sagital_4\Data');
% Dsalida= load('datos_salida.mat');
Dentrada=load('datos_entrada.mat');

Datos=Dentrada;

Ytr=spline(1:size(Datos.Data{1,1}.trayectoria,2), Datos.Data{1,1}.trayectoria, linspace(1,size(Datos.Data{1,1}.trayectoria,2),size(Datos.Data{1,1}.fuerza,2)));

F1=[];
F2=[];

for n=1:size(Datos.Data{1,1}.fuerza,2)

for i=1:7
configuration(i).JointPosition=Ytr(i,n);
end

J=geometricJacobian(robot,configuration,'iiwa_link_ee');

pseudoinvJ1=inv(J*J')*J;  %En matlab se traspone al reves
%pseudoinvJ2=(J*J')\J;

F1(:,n)=pseudoinvJ1*Datos.Data{1,1}.fuerza(:,n);
F2(:,n)=pinv(J')*Datos.Data{1,1}.fuerza(:,n);

torque_joints1(:,n)=J'*F1(:,n);  %En matlab se traspone al reves
torque_joints2(:,n)=J'*F2(:,n);
end
cd('D:\User\escrit\Universidad\Master\2\TFM\Codigos Mar\MyCode\MyCode');
[error_absoluto1,error_relativo1]=error_calculate(Datos.Data{1,1}.fuerza, torque_joints1);
[error_absoluto2,error_relativo2]=error_calculate(Datos.Data{1,1}.fuerza, torque_joints2);