q_ini = [1.29802560078651,1.49622838219569,1.42179220417360,1.56107442398227,1.42245731662296,1.52754657738387,1.34452235615645];
id_output=1;
tStartTraj = tic;
q_curr=q_ini;
control_step_size = rosparam("get", "/iiwa_command/control_step_size");
while(true)
    timeFromStart=toc(tStartTraj);
    id_curr = round(timeFromStart/control_step_size)+1;
    if (id_output~=1)
        q_exp = q_comm(id_output-1,:);
        q_curr = q_exp;
    end
    if (id_curr >= size(traj_theory.x, 1))
        disp('last iter');
        id_curr=size(traj_theory.x, 1);
        break;
    end
    x_exp = traj_theory.x(id_curr, :);
    xdot_exp = traj_theory.xdot(id_curr, :);
    x_curr = IiwaScrewTheory.ForwardKinematics(q_curr);
    xdot_exp_S = IiwaScrewTheory.tfscrew_A2S(xdot_exp, x_exp);
    %Add xdot_err
    xdot_S = xdot_exp_S;
    qdot_comm = IiwaScrewTheory.IDK_point(q_curr, xdot_S);
    %Limit qdot_comm
    q_comm(id_output,:) = q_curr + qdot_comm*control_step_size;
    x_output(id_output, :) = IiwaScrewTheory.ForwardKinematics(q_comm(id_output,:));
    id_output=id_output+1;
end