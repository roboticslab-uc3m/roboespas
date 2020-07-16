function [FT, t_i] = IiwaForwardDynamics(Trial)
    q_torque_ref = Trial.Trajectory.Reference.JointTorques.Torques;
    q_torque_trial = Trial.Trajectory.Trial.JointTorques.Torques;
    
    t_torque_ref = Trial.Trajectory.Reference.JointTorques.Timestamps;
    t_torque_trial = Trial.Trajectory.Trial.JointTorques.Timestamps;
    
    ts_ref = timeseries(q_torque_ref, t_torque_ref);
    ts_trial = timeseries(q_torque_trial, t_torque_trial);
    
    [ts_ref_sincro, ts_trial_sincro] = synchronize(ts_ref, ts_trial, 'Intersection');
    
    diff_torques = reshape(ts_ref_sincro.Data-ts_trial_sincro.Data, size(ts_ref_sincro.Data,3), 7);
    [~,~,~, Twist, ~,~] = data_IIWA();
    FT = zeros(6, size(ts_ref_sincro.Data,3));
    t = ts_ref_sincro.Time;
    for i=1:size(ts_ref_sincro.Data,3)
        t_i = t(i);
        [~, id_q] = min(abs(Trial.Trajectory.Trial.Timestamps-t_i));
        current_joint_pos=Trial.Trajectory.Trial.JointTrajectory(:,id_q);
        TwMag=[Twist; current_joint_pos'];
        J=GeoJacobianS(TwMag);
        FT(:,i)=pinv(J')*diff_torques(i,:)';
    end
end

