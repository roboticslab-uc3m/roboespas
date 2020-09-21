function [StartTime,LastTime] = fCinInv(CD_model,CD_out,model,TrcTable,OutputMotionStr,ImuMotionStr)

% INPUTS
import org.opensim.modeling.*
cd(CD_out)
% model = Model(Modelo);


trcfile=strcat(CD_model,"\CCartesianas\",TrcTable);
TRC= org.opensim.modeling.Storage(trcfile);
StartTime=TRC.getFirstTime;
LastTime=TRC.getLastTime;
%Main
% TRCFileFixer(trcfile) %No se si hace falta

IK_tool=InverseKinematicsTool('IK_setup.xml');
IK_tool.setModel(model);
IK_tool.setStartTime(StartTime);
IK_tool.setEndTime(LastTime);
IK_tool.setMarkerDataFileName(trcfile);
IK_tool.setCoordinateFileName([CD_model, '\IKResults\', ImuMotionStr]);
IK_tool.setOutputMotionFileName(OutputMotionStr);
IKmarkerTasks=IK_tool.getIKTaskSet;
IK_tool.print('IK.xml'); 

% IK=InverseKinematicsTool('IK.xml');
IK_tool.run();
strcat('Cinemática inversa calculada y guardada -> ', OutputMotionStr, 'TRC: ',trcfile)
end

