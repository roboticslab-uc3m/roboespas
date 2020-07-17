function [modelOut,MODELOOut] = f_scaleModel(MODELO,lastTime,mass,CD_model)
% Esta función está realizada para escalar a Eduardo Sáenz, si desea
% escalar a otra persona será necesario cambiar la masa.
cd(strcat(CD_model,'\CCartesianas'));
modelFileName=strcat(CD_model,'\',MODELO);
timeRange=org.opensim.modeling.ArrayDouble();   
    timeRange.append(0);
    timeRange.append(lastTime);
markerSetFileName=strcat(CD_model,'\CCartesianas\Lab.trc');

scaleTool=org.opensim.modeling.ScaleTool(strcat(CD_model,'\CCartesianas\Scale_setup_KinectData.xml'));
scaleTool.setSubjectMass(mass);
% genericModelMaker=scaleTool.getGenericModelMaker;
    scaleTool.getGenericModelMaker.setModelFileName(modelFileName);
    scaleTool.getGenericModelMaker.setMarkerSetFileName(markerSetFileName);
    scaleTool.getModelScaler.setMarkerFileName(markerSetFileName);
    scaleTool.getModelScaler.setTimeRange(timeRange);
    scaleTool.getModelScaler.setOutputModelFileName(regexprep(modelFileName,'.osim',' - scaled.osim'));

    scaleTool.print('Scale_setup_actual.xml');
    tool=org.opensim.modeling.ScaleTool('Scale_setup_actual.xml');
tool.run;

    modelFileNameOut=regexprep(modelFileName,'.osim',' - scaled.osim');
    modelOut=org.opensim.modeling.Model(modelFileNameOut);
    MODELOOut=regexprep(MODELO,'.osim',' - scaled.osim');

end

