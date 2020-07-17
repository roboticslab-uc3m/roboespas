function [cmcResults, modelStates] = f_importCMCResults(sto)
% This function saves in a .mat struct the estimated results after CMC
% completion
    results = importdata(sto);
    for i = 1:length(results.colheaders)
        varName = results.colheaders{i};
        if i==1
            cmcResults.(varName) = results.data(:,1);
        else
            if ~contains(sto, 'states')
                cmcResults.(varName) = results.data(:,i);
            else
                if contains(varName, 'forceset') && contains(varName, 'activation')
                    muscleName = extractBetween(varName,'/forceset/','/activation');
                    cmcResults.(muscleName{1}) = results.data(:,i);
                else
                    if contains(varName, 'forceset')
                        muscleName = extractBetween(varName,'/forceset/','/fiber_length');
                        modelStates.(muscleName{1}) = results.data(:,i);
                    else
                        val = find(varName =='/');
                        modelStates.(varName(val(end-1)+1:val(end)-1)).(varName(val(end)+1:end)) = results.data(:,i);
                    end
                end
            end
        end
    end
end
