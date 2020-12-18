
figHandles = get(groot, 'Children');

folder = '/home/roboespas/roboespas/Results/FRIResults/CommandModes/MoveJTrajCartVelControl/V60/k0_003/';
names = {'q_command', 'q_precision', 'q_repeatibility', 'qd_command', 'qd_precision', 'qd_repeatibility', 'x_command', 'x_precision', 'x_repeatibility', 'xd_command', 'xd_precision', 'xd_repeatibility'};
for i=1:length(figHandles)
    id = figHandles(i).Number;
    saveas(figHandles(i), [folder, names{id}, '.png']);    
    saveas(figHandles(i), [folder, names{id}, '.fig']);    
end
trajs_trials = obj.trajs_trial

close all