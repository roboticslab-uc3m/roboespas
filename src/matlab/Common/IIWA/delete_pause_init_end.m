function data_output = delete_pause_init_end(data)
    %Does not affect pp field, better apply before or inside bounded_spline function
    minindex=size(data.t,2);
    maxindex=1;
    mincartvelocity=0.015; %m/s
    xdot_euc_pos=vecnorm(data.xdot.pos);
    xdot_euc_ori=vecnorm(data.xdot.ori);
    indices=find(xdot_euc_pos>mincartvelocity);
    if (~isempty(indices))
        minindex=indices(1);
        maxindex=indices(end);
    else
        minindex=1;
        maxindex=size(data.t,2);
    end
    data_output.t=data.t(minindex:maxindex);
    if (~isempty(data_output.t))
        data_output.t=data_output.t-data_output.t(1);
        data_output.q=data.q(:,minindex:maxindex);
        data_output.qdot=data.qdot(:,minindex:maxindex);
        data_output.qdotdot=data.qdotdot(:,minindex:maxindex);
        data_output.x.pos=data.x.pos(:,minindex:maxindex);
        data_output.x.ori=data.x.ori(:,minindex:maxindex);
        data_output.xdot.pos=data.xdot.pos(:,minindex:maxindex);
        data_output.xdot.ori=data.xdot.ori(:,minindex:maxindex);
        
        if (isfield(data, 'q_torque'))
            data_output.q_torque=data.q_torque;
        end
        if (isfield(data, 't_torque'))
            data_output.t_torque=data.t_torque;
        end
    else
        disp('Couldnt capture robot movement')
    end
end

