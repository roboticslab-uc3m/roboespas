function [cmcExcitations, others] = f_importCMCExcitations(sto)
% This function saves in a .mat struct the estimated excitations after CMC
% completion
    results = importdata(sto);
    for i = 1:length(results.colheaders)
        varName = results.colheaders{i};
        if i==1
            cmcExcitations.(varName) = results.data(:,1);
        else
            if contains(sto, 'controls') || contains(sto, 'force') || contains(sto, 'power') || contains(sto, 'speed') || contains(sto, 'pErr')
                cmcExcitations.(varName) = results.data(:,i);
            else
                if contains(varName, 'forceset') && contains(varName, 'activation')
                    muscleName = extractBetween(varName,'/forceset/','/activation');
                    cmcExcitations.(muscleName{1}) = results.data(:,i);
                else
                    if contains(varName, 'forceset')
                        muscleName = extractBetween(varName,'/forceset/','/fiber_length');
                        others.(muscleName{1}) = results.data(:,i);
                    else
                        val = find(varName =='/');
                        others.(varName(val(end-1)+1:val(end)-1)).(varName(val(end)+1:end)) = results.data(:,i);
                    end
                end
            end
        end
    end
end

