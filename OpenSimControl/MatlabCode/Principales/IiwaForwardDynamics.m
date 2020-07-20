function [FT, t] = IiwaForwardDynamics(Trial)
    % Función implementada para calcular la dinámica directa del IIWA. Para ello se comparan las fuerzas realizadas por
    % cada articulación del robot de referencia y durante el trial.

    q_torque_ref = Trial.Trajectory.Reference.JointTorques.Torques;
    q_torque_trial = Trial.Trajectory.Trial.JointTorques.Torques;

    t_torque_ref = Trial.Trajectory.Reference.JointTorques.Timestamps;
    t_torque_trial = Trial.Trajectory.Trial.JointTorques.Timestamps;

    ts_ref = timeseries(q_torque_ref, t_torque_ref);
    ts_trial = timeseries(q_torque_trial, t_torque_trial);

    [ts_ref_sincro, ts_trial_sincro] = synchronize(ts_ref, ts_trial, 'Intersection');

    torquesRef = reshape(ts_ref_sincro.Data, 7, size(ts_ref_sincro.Data,3));
    torquesTrial = reshape(ts_trial_sincro.Data, 7, size(ts_trial_sincro.Data,3));
    t = ts_ref_sincro.Time;
    diff_torques = torquesTrial-torquesRef;

    [~,~,~, Twist, ~,~] = data_IIWA();
    FT = zeros(size(t,1), 6);
    for i=1:size(t,1)
        t_torque = ts_ref_sincro.Time(i);
        [~, id_q] = min(abs(Trial.Trajectory.Trial.Timestamps-t_torque));
        current_joint_pos=Trial.Trajectory.Trial.JointTrajectory(:,id_q);
        TwMag=[Twist; current_joint_pos'];
        J=GeoJacobianS(TwMag);
        FT(i,:)=pinv(J')*diff_torques(:,i);
    end
end

