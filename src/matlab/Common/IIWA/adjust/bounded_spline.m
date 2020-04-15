function data_output = bounded_spline(data_input, velocity, smoothing)
    if (isempty(data_input))
        data_output=[];
        return;
    end
    [q_out, qdot_out, qdotdot_out, tKnot, q_coefs, qdot_coefs, qdotdot_coefs]=boundedspline(data_input.t, data_input.q, velocity, smoothing);
    % Fill output
    %data_output.qdot_previous=data_input.qdot;
    data_output.pp.breaks=tKnot;
    data_output.pp.coefs.q=q_coefs;
    data_output.pp.coefs.qdot=qdot_coefs;
    data_output.pp.coefs.qdotdot=qdotdot_coefs;
    data_output.q=q_out;
    data_output.qdot=qdot_out;
    data_output.qdotdot=qdotdot_out;
    data_output.t=data_input.t;
    data_output=fill_cartesian(data_output);
    if (isfield(data_input, 'q_torque'))
        data_output.q_torque=data_input.q_torque;
    end
    if (isfield(data_input, 't_torque'))
        data_output.t_torque=data_input.t_torque;
    end
end
