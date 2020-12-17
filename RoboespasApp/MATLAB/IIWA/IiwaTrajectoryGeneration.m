classdef IiwaTrajectoryGeneration
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    methods (Static, Access = 'public')
        function traj_output = TrapezoidalVelocityProfileTrajectory(q_ini, q_goal, control_step_size, qdot, qdotdot, name)
            qinc = q_goal-q_ini;
            npoints = 1000;
            vels_normalised = qdot./qinc;
            [qdot_perc, j_limits_v] = max(abs(1./vels_normalised));
            qdot_used = qinc./qdot_perc;
            accs_normalised = qdotdot./qinc;
            [qdotdot_perc, j_limits_a] = max(abs(1./accs_normalised));
            qdotdot_used = qdot_used./qdotdot_perc;
            v_normalised = abs(vels_normalised(j_limits_v));
            a_normalised = abs(accs_normalised(j_limits_a));
            try
                [s, sd, sdd, ts] = trapveltraj([0 1], npoints, 'PeakVelocity', v_normalised, 'Acceleration', a_normalised/10);
            catch
                [s, sd, sdd, ts] = trapveltraj([0 1], npoints);
            end
            traj_output = IiwaTrajectory(name,npoints);
            traj_output.t=ts;
            traj_output.q=q_ini + s'.*qinc;
            traj_output = traj_output.CompleteVelAcc();
            traj_output = traj_output.ChangeSampleTime(control_step_size);
        end
    end
end

