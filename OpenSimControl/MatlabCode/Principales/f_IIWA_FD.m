function [ForceAndTorque,stamps] = f_IIWA_FD(Datos,DatosVacio,tsample)
% Función implementada para calcular la dinámica directa del IIWA en
% movimientos pasivos. Para ello se comparan las fuerzas realizadas por
% cada articulación del robot con y sin paciente.

torquesVacio=DatosVacio{1}.fuerza;
stampsVacio=DatosVacio{1}.stamps_fuerza;
[stamps, torquesVacio]=equidistant(stampsVacio, torquesVacio, tsample);
clear flexion_codo_rapida_vacio;

torquesCargado=Datos{1}.fuerza;
torqueStampsCargado=Datos{1}.stamps_fuerza;
[~, torquesCargado]=equidistant(torqueStampsCargado, torquesCargado, tsample);

jp_cargado=Datos{1}.trayectoria;
jp_stampsCargado=Datos{1}.stamps;
[~, jp_cargado]=equidistant(jp_stampsCargado, jp_cargado, tsample);
clear flexion_codo_rapida_sinfuerza;


externalFT = FD(torquesCargado, torquesVacio, jp_cargado);

% Se cambia la direccion de los valores ya que van a actuar sobre el brazo
% del paciente y no al revés.
ForceAndTorque(:,1:3)=externalFT(1:3,:)';
ForceAndTorque(:,7:9)=externalFT(4:6,:)';

%cp_cargado = FK(jp_cargado);


end

