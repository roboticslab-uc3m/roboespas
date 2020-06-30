function [cmcExcitations] = f_importCMCExcitations(sto)
% This function saves in a .mat struct the estimated excitations after CMC
% completion
    results = importdata(sto);
    for i = 1:length(results.colheaders)
        varName = results.colheaders{i};
        if i==1
            cmcExcitations.(varName) = results.data(:,1);
        else
            if contains(sto, 'controls')
                cmcExcitations.(varName) = results.data(:,i);
            else
                if contains(varName, 'forceset') && contains(varName, 'activation')
                    muscleName = extractBetween(varName,'/forceset/','/activation');
                    cmcExcitations.(muscleName{1}) = results.data(:,i);
                end
            end
        end
    end
end

