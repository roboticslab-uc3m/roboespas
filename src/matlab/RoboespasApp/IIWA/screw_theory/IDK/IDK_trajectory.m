function data_output = IDK_trajectory(xpos, xori, t, q_ini)
    data_output.q(:,1)=q_ini;
    for i=1:size(xpos,2)-1
        [data_output.qdot(:,i), data_output.q(:,i+1)] = inverse_differential_kinematics(data_output.q(:,i)', [xpos(:, i+1); xori(:,i+1)]', t(i+1)-t(i));
    end
    data_output.qdot(:,size(xpos,2))=[0; 0; 0; 0; 0; 0; 0];
    data_output.t = t;
    data_output=fill_cartesian(data_output);
end

