function [Results] = f_AnalyzeTool(CD_model,model,MotPath,ARPath)
% Funcion para realizar un ANALYSIS TOOL. El xml solo incluye un Muscle
% Analysis, pero puede ser ampliado a otro tipo de analisis
% INPUTS: MODEL Y MOTION
disp('Inicio del análisis...');
cd(CD_model)
import org.opensim.modeling.*

results_folder = ARPath;

%xml
genericSetupForAn = fullfile('AT_4M_Setup_Analyze_MuscleAn.xml');
analyzeTool = AnalyzeTool(genericSetupForAn); % lleva metido un Muscle Analysis
name='AnalyzeResults';
%mot
motIKCoordsFile = MotPath;
motCoordsData = Storage(motIKCoordsFile);
    initial_time = motCoordsData.getFirstTime();
    final_time = motCoordsData.getLastTime();
    
    % Añado un analisis cinematico para investigar
%         Kin=Kinematics(model);
%         Kin.setModel(model);
%         Kin.set_model(model);
%         Kin.setStatesStore(motCoordsData);
%         Kin.set_statesStore(motCoordsData);
%         Kin.setStepInterval(0.001);
% 
%         Kin.setStartTime(initial_time);
%         Kin.setEndTime(final_time);
%         Kin.setInDegrees(true);
% 
%         model.addAnalysis(Kin)
%Tool
analyzeTool.setModel(model);
analyzeTool.setName(name);
analyzeTool.setResultsDir(results_folder);
analyzeTool.setCoordinatesFileName(motIKCoordsFile);
analyzeTool.setInitialTime(initial_time);
analyzeTool.setFinalTime(final_time);   

outfile = ['Setup_Analyze_' name '.xml'];
analyzeTool.print(fullfile(CD_model, outfile));

analyzeTool.run();

disp(strcat('Analisis realizado correctamente y guardado en: ', results_folder));
disp(strcat(' XML del análisis -> ',outfile));

Results=strcat(ARPath,'\',name,'_MuscleAnalysis_');
end

