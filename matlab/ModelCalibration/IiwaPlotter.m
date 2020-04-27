classdef IiwaPlotter < handle
    properties (Constant)
        ColorDesired='g'
        ColorCommanded='b'
        ColorOutput='r'
        ColorOthers='m'
    end
    
    methods
        function obj = IiwaPlotter()
        end
    end
    methods(Static)
        function effortPoint(as_feedback_msg)
           % Time between points plotted. You may need to change this depending on 
           % your computer's resources
           time_plot=0.05; %seconds
           if (mod(as_feedback_msg.TimeFromStart, time_plot)==0)
                for j=1:7
                    subplot(7,1,j);
                    plot(as_feedback_msg.TimeFromStart, as_feedback_msg.PointCommanded.Effort(j), ['.', IiwaPlotter.ColorCommanded]);
                    hold on;
                    plot(as_feedback_msg.TimeFromStart, as_feedback_msg.JointState.Effort(j), ['.', IiwaPlotter.ColorRead]);
                end
           end
        end
        function effortWithPD(traj_des, traj_comm)
            figure;
            ts_des=timeseries(traj_comm.effort, traj_comm.t);
            ts_comm=timeseries(traj_des.effort, traj_des.t);
            [ts_des, ts_comm] = synchronize(ts_des, ts_comm, 'Intersection');
            ts_pdTorque=ts_comm-ts_des;
            for j = 1:7
                subplot(7,1,j);
                plot(traj_des.t, traj_des.effort(:,j), IiwaPlotter.ColorDesired); hold on
                plot(ts_pdTorque.Time, ts_pdTorque.Data(:,j), IiwaPlotter.ColorOthers);
                plot(traj_comm.t, traj_comm.effort(:,j), IiwaPlotter.ColorCommanded);
                if j == 1
                    legend('Torque desired (N)','PD torque','Total torque');
                    title('Commanded and output efforts with PD efforts')
                end
                if j == 7
                    xlabel('time (s)');
                end
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
        end
        function joint_positions(traj_comm, traj_output)
            figure;
            for j = 1:7
                subplot(7,1,j);
                plot(traj_comm.t, traj_comm.q(:,j), IiwaPlotter.ColorCommanded); 
                hold on
                plot(traj_output.t, traj_output.q(:,j), IiwaPlotter.ColorOutput); 
                if j == 1
                    legend('Commanded joint position (rad)','Output joint position');
                    title('Commanded and output joint positions')
                end
                if j == 7
                    xlabel('time (s)');
                end
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
           
        end
        function joint_efforts(traj_comm, traj_output)
            figure;
            for j=1:7
                subplot(7,1,j);
                plot(traj_comm.t, traj_comm.effort(:,j), IiwaPlotter.ColorCommanded);
                hold on;
                plot(traj_output.t, traj_output.effort(:,j), IiwaPlotter.ColorOutput);
                if j == 1
                    legend('Commanded joint effort(N)','Output joint effort');
                    title('Commanded and output efforts')
                end
                if j == 7
                    xlabel('time (s)');
                end
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
        end
    end
end

