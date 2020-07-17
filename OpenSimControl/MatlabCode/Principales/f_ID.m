function f_ID(CD,model,OutputMotionStr,OutputIDStr,StartTime,LastTime)
cd(CD)

import org.opensim.modeling.*

disp('Calculando DINAMICA INVERSA, por favor, espere');

ID_tool=InverseDynamicsTool();
ID_tool.setModel(model);
ID_tool.setStartTime(StartTime);
ID_tool.setEndTime(LastTime);
% tool.setMarkerDataFileName('Lab.trc')
ID_tool.setCoordinatesFileName(OutputMotionStr)
ID_tool.setOutputGenForceFileName(OutputIDStr)
ID_tool.run();

disp(strcat('Dinámica inversa calculada y guardada -> ', OutputIDStr));
end

