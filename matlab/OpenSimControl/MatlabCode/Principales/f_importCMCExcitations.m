function [cmcExcitations] = f_importCMCExcitations(controlsSTO)
% This function saves in a .mat struct the estimated excitations after CMC
% completion
    controls = importdata(controlsSTO);
    for i = 1:length(controls.colheaders)
        varName = controls.colheaders{i};
        cmcExcitations.(varName) = controls.data(:,i);
    end
end

