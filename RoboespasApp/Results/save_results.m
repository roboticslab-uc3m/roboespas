% %q_command
% IiwaPlotter.fill_joint_position(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
% IiwaPlotter.joint_positions({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials, obj.ColorTrials, obj.ColorCommanded, '--'}, display)
% %q_precision
% IiwaPlotter.fill_joint_position(obj.traj_min_error, obj.traj_max_error, obj.ColorTrials, display);
% IiwaPlotter.joint_positions({obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
% IiwaPlotter.joint_position_rms(obj.traj_mean_error, obj.ColorTrials, display);
% %q_repeatibility
% IiwaPlotter.fill_joint_position(obj.traj_min_error_rep, obj.traj_max_error_rep, obj.ColorTrials, display);
% IiwaPlotter.joint_positions({obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
% IiwaPlotter.joint_position_rms(obj.traj_mean_error_rep, obj.ColorTrials, display);
% %qd_command
% IiwaPlotter.fill_joint_velocity(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
% IiwaPlotter.joint_velocities({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials,obj.ColorTrials, obj.ColorCommanded, '--'}, display)
% %qd_precision
% IiwaPlotter.fill_joint_velocity(obj.traj_min_error, obj.traj_max_error, obj.ColorTrials, display);
% IiwaPlotter.joint_velocities({obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
% IiwaPlotter.joint_velocity_rms(obj.traj_mean_error, obj.ColorTrials, display);
% %qd_repeatibility
% IiwaPlotter.fill_joint_velocity(obj.traj_min_error_rep, obj.traj_max_error_rep, obj.ColorTrials, display);
% IiwaPlotter.joint_velocities({obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep}, {obj.ColorTrials, 'k', obj.ColorTrials}, display) 
% IiwaPlotter.joint_velocity_rms(obj.traj_mean_error_rep, obj.ColorTrials, display);
% %x_command
% IiwaPlotter.fill_cartesian_position(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
% IiwaPlotter.cartesian_positions({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials, obj.ColorTrials, obj.ColorCommanded, '--'}, display)
% %x_precision
% IiwaPlotter.cartesian_repeatibility_position_error(obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error,  display)
% %x_repeatibility
% IiwaPlotter.cartesian_repeatibility_position_error(obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep,  display)
% %xd_command
% IiwaPlotter.fill_cartesian_velocity(obj.traj_min, obj.traj_max, obj.ColorTrials, display);
% IiwaPlotter.cartesian_velocities({obj.traj_min, obj.traj_mean, obj.traj_max, obj.traj_command}, {obj.ColorTrials, obj.ColorTrials, obj.ColorTrials, obj.ColorCommanded, '--'}, display)
% %xd_precision
% IiwaPlotter.cartesian_repeatibility_velocity_error(obj.traj_min_error, obj.traj_mean_error, obj.traj_max_error,  display)
% %xd_repeatibility
% IiwaPlotter.cartesian_repeatibility_velocity_error(obj.traj_min_error_rep, obj.traj_mean_error_rep, obj.traj_max_error_rep,  display)

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