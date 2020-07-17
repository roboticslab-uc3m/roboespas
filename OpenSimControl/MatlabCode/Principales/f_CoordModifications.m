function [Handle] = f_CoordModifications(Datos)
%Esta funcion permite rotar y cambiar el offset de las coordenadas
%recibidas desde el laboratorio

% Coordenadas de la mano corregida unicamente la orientacion, ya que
% supongo que se realiza en un plano aproximadamente paralelo al del robot
posicion(:,1)=-Datos.Handle(:,2);
posicion(:,2)=Datos.Handle(:,3);
posicion(:,3)=-Datos.Handle(:,1);
% Se denomina Handle a la posición proporcionada por el TCP del robot y que
% se asume como posición de la mano en la simulación.

%Una vez corregida la orientacion calculo los offsets
posWristX=mean(Datos.WristRight(1:10,1));
posWristY=mean(Datos.WristRight(1:10,2));
posWristZ=mean(Datos.WristRight(1:10,3));
posHandleX=mean(posicion(1:10,1));
posHandleY=mean(posicion(1:10,2));
posHandleZ=mean(posicion(1:10,3));

diffX=posHandleX-posWristX;
diffY=posHandleY-posWristY;
diffZ=posHandleZ-posWristZ;

%Correccion de las coordenadas del Handle
    x=posicion(:,1)-diffX+0.004;
    y=posicion(:,2)-diffY-0.045;
    z=posicion(:,3)-diffZ+0.00237;

          Handle(:,1)=x;
          Handle(:,2)=y;
          Handle(:,3)=z;
      
end

