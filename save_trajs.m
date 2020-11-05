traj_command = obj.traj_command;
traj_mirrored = obj.traj_mirrored;
trajs_trial = obj.trajs_trial;

v='15';
k='0';
folder = ['/home/roboespas/roboespas/Results/FRIResults/CommandModes/MoveJTrajCartVelControl/V', v, '/k', k,'/'];
save([folder, 'traj_command.mat'], 'traj_command');
save([folder, 'traj_mirrored.mat'], 'traj_mirrored')
save([folder, 'trajs_trial.mat'], 'trajs_trial');