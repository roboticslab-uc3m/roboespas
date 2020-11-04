load('/home/roboespas/roboespas/Common/TestFRI/TestCartVelControl/traj_des.mat')

close all;
rosparam('set', '/iiwa_command/control_cart_vel/kp', 0);
[traj_comm, traj_out] = IiwaCommandFRI.MoveJCartVelTraj(traj_des);
IiwaPlotter.cartesian_position_error(traj_des, traj_out);


rosparam('set', '/iiwa_command/control_cart_vel/kp', 0.001);
[traj_comm, traj_out] = IiwaCommandFRI.MoveJCartVelTraj(traj_des);
IiwaPlotter.cartesian_position_error(traj_des, traj_out);

rosparam('set', '/iiwa_command/control_cart_vel/kp', 0.005);
[traj_comm, traj_out] = IiwaCommandFRI.MoveJCartVelTraj(traj_des);
IiwaPlotter.cartesian_position_error(traj_des, traj_out);

rosparam('set', '/iiwa_command/control_cart_vel/kp', 0.01);
[traj_comm, traj_out] = IiwaCommandFRI.MoveJCartVelTraj(traj_des);
IiwaPlotter.cartesian_position_error(traj_des, traj_out);