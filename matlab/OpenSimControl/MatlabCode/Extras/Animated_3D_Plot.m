clear all
close all
clc

%Open the directory where .osim model is located
cd('D:\User\escrit\Universidad\Master\2\TFM\Movimientos\plano_sagital_hombro\Data');
Dentrada=load('datos_entrada.mat');
Posicion=Dentrada.Data{1, 1}.endeffector_posicion;
%  posicion=Posicion(:,1:200);
posicion=Posicion;
 xS=(posicion(1,:));
 yS=posicion(2,:);
 zS=posicion(3,:);
cd('D:\User\escrit\Universidad\Master\2\TFM\Movimientos\Codo sagital\codo_sagital_4\Data');
Dsalida= load('datos_salida.mat');
Dentrada=load('datos_entrada.mat');
Drecalculados=load('datos_recalculados.mat');

Posicion=Dentrada.Data{1, 1}.endeffector_posicion;
%  posicion=Posicion(:,1:200);
posicion=Posicion;
     x=zeros(1,length(posicion(1,:)));
     y=zeros(1,length(posicion(2,:)));
     z=zeros(1,length(posicion(3,:)));
 
     xOld=posicion(1,:)-0.58;
     yOld=posicion(2,:);
     zOld=posicion(3,:)-0.5;
     
      ax=pi/4;
      ay=0;
      az=0;
      Rx=[1 0 0;0 cos(ax) -sin(ax);0 sin(ax) cos(ax)];
      Ry=[cos(ay) 0 sin(ay);0 1 0;-sin(ay) 0 cos(ay)];
      Rz=[cos(az) -sin(az) 0;sin(az) cos(az) 0;0 0 1];

          for i=1:length(x)
             V=[xOld(i) yOld(i) zOld(i)];
             Vr= V*Rx*Ry*Rz;
                x(i)=Vr(1)+0.2;
                y(i)=Vr(2);%-0.5;
                z(i)=Vr(3)-0.2;
          end
            
% z = 0:0.05:10;
% y = sin(2*z);
% x = cos(2*z);
%% 2D
% i=length(x);
% figure(1)
% hold on
%     plot(x,z,'r')
%     plot(x(1),z(1),'o')
%     plot(x(i),z(i),'x')
%    
%     figure(2)
%      hold on
%     plot(x,y,'g')
%     plot(x(1),y(1),'o')
%     plot(x(i),y(i),'x')
%   
%     figure(3)
%      hold on
%     plot(y,z,'b')
%     plot(y(1),z(1),'o')
%     plot(y(i),z(i),'x')
    
    
%% 3D LINE
% cuerpo:
xc=[0 0 0]';
yc=[0 -0.15 0]';
zc=[0 0 -0.3]';
    figure(5)
    hold on
    plot3(xc,yc,zc);
    axis equal
    hold on

  plot3(x,y,z) ;
  
  xlabel('x','FontSize',10,'FontWeight','bold','Color','r');
  ylabel('y','FontSize',10,'FontWeight','bold','Color','r');
  zlabel('z','FontSize',10,'FontWeight','bold','Color','r');
  hold on
%   plot3(xS,yS,zS) ;
  axis equal
    
 




%% 3D animated
% figure(4)
% xGrid='on';
% yGrid='on';
% zGrid='on';
% curve = animatedline('LineWidth',5);
% puntos = animatedline('LineWidth',10);
% set(gca,'XLim',[0 1],'YLim',[-1 0],'ZLim',[-0.2 1]);
% view(43,24);
% hold on;
% addpoints(puntos,0,0,-0.15);
% addpoints(puntos,0,0,0);
% addpoints(puntos,0,-0.3,0);
% 
% for i=1:length(z)
%     i
%     addpoints(curve,x(i),y(i),z(i));
% %     head = scatter3(x(i),y(i),z(i),'filled','MarkerFaceColor','b');
%     drawnow
%     pause(0.01);
% %     delete(head);
% end
% 
