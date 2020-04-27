function data = fill_cartesian(data)
% Function that fills the fields data.x.ori, data.x.pos, data.xdot.pos and
% data.xdot.ori of a given data struct, by calculating its forward
% kinematics and then calculating cartesian velocities as the substraction
% of contiguous cartesian positions. It takes into account that cartesian
% velocities in orientation cannot be calculating by substraction, but by
% transforming orientations into rotation matrices first.
    % Initializate variables in case they were already filled, do not
    % conserve the size of the previous ones
    data.x.ori=[];
    data.x.pos=[];
    data.xdot.pos=[];
    data.xdot.ori=[];
    [data.x.pos, data.x.ori]=forward_kinematics_mex(data.q);
    data.xdot.pos=[(data.x.pos(:,2:end)-data.x.pos(:,1:end-1))./(data.t(2:end)-data.t(1:end-1)) [0;0;0]];
    %Cartesian velocity in orientation cannot be calculated by substraction
    data.xdot.ori=substract_ori_traj_mex(data.x.ori, data.t);
    data.xdot.ori=[data.xdot.ori [0; 0; 0]];
end

