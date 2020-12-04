function traj = Data2traj(Data)
    traj = IiwaTrajectory();
    traj.q = Data.q';
    traj.qdot = Data.qdot';
%    traj.qdotdot = Data.qdotdot';
    traj.t = Data.t';
    traj.x = [Data.x.pos(1:3,:); Data.x.ori(1:3,:)]';
    traj.xdot = [Data.x.pos(1:3,:); Data.x.ori(1:3,:)]';
%     if(~isempty(Data.q_torque))
%         traj.effort = Data.q_torque(:,1:min(length(Data.q_torque), length(Data.q)));
%     end
    traj.name = inputname(1);
    traj.npoints = size(traj.q,1);
end

