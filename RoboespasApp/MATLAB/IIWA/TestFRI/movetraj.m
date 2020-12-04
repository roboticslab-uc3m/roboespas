init_ros;
load('/home/roboespas/roboespas/Common/Trajectories/flexext_P004_R_201013_161218_adj/adjusted.mat');
%Loaded traj and angvel
w_obj = 90;
middle_pause = 2;
smoothing_command = 1e-8;
seg_command = 100;
css = IiwaCommandFRI.GetControlStepSize();
traj_velocity_circle = traj.ChangeVelocity(w_obj/angvel);
traj_spline = traj_velocity_circle.BoundedSpline(smoothing_command, seg_command);
traj_spline_pause = traj_spline.AddPause(middle_pause);
traj_mirrored = traj_spline_pause.MergeAfterwards(traj_spline.MirrorTrajectory());
traj_mirrored.name = 'mirrored';
traj_command = traj_mirrored.ChangeSampleTime(css);
traj_command.name = 'command';


IiwaCommandFRI.SetVelocity(0.2);
[traj_comm_movej, traj_out_movej, traj_theory_movej] = IiwaCommandFRI.MoveJ(traj_command.q(1,:));
pause(1);

[traj_comm, traj_out] = IiwaCommandFRI.MoveJTraj(traj_command);

%traj_theory = traj.ChangeSampleTime

%[traj_theory, traj_comm, traj_read] = IiwaCommandFRI.MoveJ(qend);
