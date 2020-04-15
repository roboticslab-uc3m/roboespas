function data_output = fill_cartesian_withoutmex(data)
% Function that fills the fields data.x.ori, data.x.pos, data.xdot.pos and
% data.xdot.ori of a given data struct, by calculating its forward
% kinematics and then calculating cartesian velocities as the substraction
% of contiguous cartesian positions. It takes into account that cartesian
% velocities in orientation cannot be calculating by substraction, but by
% transforming orientations into rotation matrices first.
    % Initializate variables in case they were already filled, do not
    % conserve the size of the previous ones
    data_output=struct('pp', data.pp, 't', data.t, 'q', data.q, 'qdot', data.qdot, 'qdotdot', data.qdotdot, 'x', struct('ori',zeros(3,size(data.t,2)), 'pos', zeros(3,size(data.t,2))), 'xdot', struct('ori',zeros(3,size(data.t,2)), 'pos', zeros(3,size(data.t,2))));
    %data_output.x.ori=zeros(3,size(data_output.t,2));
    %data_output.x.pos=zeros(3,size(data_output.t,2));
    %data_output.xdot.pos=zeros(3,size(data_output.t,2));
    %data_output.xdot.ori=zeros(3,size(data_output.t,2));
    [data_output.x.pos, data_output.x.ori]=forward_kinematics(data_output.q);
    len=size(data.t,2);
    t_substract=(data.t(2:end)-data.t(1:end-1));
    xpos_substract=(data_output.x.pos(:,2:end)-data_output.x.pos(:,1:end-1));
    data_output.xdot.pos(:, 1:end-1)=rdivide((xpos_substract(1:3,1:len-1)),(t_substract(1,1:len-1)));
    %Cartesian velocity in orientation cannot be calculated by substraction
    data_output.xdot.ori(:, 1:end-1)=substract_ori_traj(data_output.x.ori, data_output.t);
end

