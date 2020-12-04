init_ros;
qini = IiwaCommandFRI.ReadCurrentJointPosition();
qend = qini;
qend(7) = qend(7)+deg2rad(20);
IiwaCommandFRI.SetVelocity(0.1);
[traj_theory, traj_comm, traj_read] = IiwaCommandFRI.MoveJ(qend);
