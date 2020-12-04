function Data = traj2Data(traj)
    Data.q = traj.q';
    Data.qdot = traj.qdot';
    Data.qdotdot = traj.qdotdot';
    Data.t = traj.t';
    Data.x.pos = traj.x(:,1:3)';
    Data.x.ori = traj.x(:,4:6)';
    Data.xdot.pos = traj.xdot(:,1:3)';
    Data.xdot.ori = traj.xdot(:,4:6)';
    Data.q_torque = traj.effort';
    Data.t_torque = traj.t';
end

