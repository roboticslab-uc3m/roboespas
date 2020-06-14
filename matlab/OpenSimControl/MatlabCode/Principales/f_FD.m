function [FDOutputPath] = f_FD(model,New_Storage,New_Path,CD_model,External_Loads)

import org.opensim.modeling.*

    initial_time = New_Storage.getFirstTime();
    final_time = New_Storage.getLastTime();

name='FDOutput';
FDOutputPath= strcat(CD_model,'\FD_Output\');
    
    %% Si quiero añadir un analisis (MuscleAnalysis y Kinematics en este caso):
    %MA
%         % Para luego modificarlo o eliminarlo:
%         model.getNumAnalyses;
%         AS=model.getAnalysisSet;
%         MuscleAnalysis=AS.get(0);
%         model.removeAnalysis(MuscleAnalysis);
%         model.getNumAnalyses; % = 0

% MA=MuscleAnalysis();
% MA.setModel(model);
% % MA.set_model(model);
% MA.setStatesStore(New_Storage);
% % MA.set_statesStore(motData);
% MA.setStartTime(initial_time);
% MA.setEndTime(final_time);
% MA.setInDegrees(true);
%     model.addAnalysis(MA)
% 
% 
Kin=Kinematics();
% Kin.setModel(model);
Kin.set_model(model);
% Kin.setStatesStore(New_Storage);
Kin.set_statesStore(New_Storage);
Kin.setStepInterval(0.001);%?
Kin.setStartTime(initial_time);
Kin.setEndTime(final_time);
Kin.setInDegrees(true);
    model.addAnalysis(Kin)
%% FD Tool
    tool = ForwardTool(); 
tool.setName(name);
tool.setModel(model);
tool.setStartTime(initial_time);
tool.setFinalTime(final_time);
% tool.setControlsFileName(New_Path);%???????? Valores modificados
tool.setResultsDir(FDOutputPath);
% if ~ischar(External_Loads)
tool.setExternalLoads(External_Loads);
% end
% tool.setStatesFileName(New_Path); %????????

tool.run(); 
end

