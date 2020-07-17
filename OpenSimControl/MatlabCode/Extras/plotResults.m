function plotResults(filename)
    controls = f_importCMCExcitations(filename);
    muscles = fieldnames(controls);
    numMuscles = length(muscles)-1;
    f = figure('WindowState', 'maximized');
    for i = 1:numMuscles
        subplot(ceil(numMuscles/2),2,i, 'Parent',f);
        hold on
        plot(controls.time, ones(length(controls.time),1)*rms(controls.(muscles{i+1})),'Color','r', 'LineStyle', "--")
        plot(controls.time, controls.(muscles{i+1}), 'Color', 'b')
        title(replace(muscles{i+1}, '_', ' '))
        legend(['RMS ',num2str(rms(controls.(muscles{i+1}))), ' '])
        %     ylim([0 2])
        drawnow
    end
