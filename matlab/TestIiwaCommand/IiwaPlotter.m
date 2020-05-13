classdef IiwaPlotter < handle
    properties (Constant)
        ColorDesired='g'
        ColorCommanded='b'
        ColorOutput='r'
        ColorOthers='m'
        ColorErrors='r'
        TimePlot=0.05;
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
        function effortWithPD_compare(traj_des, traj_comm)
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
        function effortWithPD_compare_big(traj_comm, traj_output, traj_withoutPD)
            for i=1:7
                figure;hold on;
                plot(traj_comm.t,traj_comm.effort(:,i), IiwaPlotter.ColorCommanded);
                plot(traj_withoutPD.t,traj_withoutPD.effort(:,i), IiwaPlotter.ColorOthers);
                plot(traj_output.t,traj_output.effort(:,i), IiwaPlotter.ColorOutput);
                legend('commanded', 'ideal','output');
                title('Commanded and output efforts with PD efforts')
                xlabel('time (s)');
                ylabel(s);
            end
        end
        function joint_position(joint_position, t)
            time_plot=0.05; %seconds
            if (mod(t, time_plot)==0)
                for j = 1:7
                    subplot(7,1,j);
                    hold on
                    plot(t, joint_position(:,j), [IiwaPlotter.ColorCommanded, '.']); 
                    if j == 7
                        xlabel('time (s)');
                    end
                    s = sprintf('j%d',j);
                    ylabel(s);
                    grid on;
                end
            end
        end
        function cartesian_position(cartesian_position, t)
            time_plot=0.05;
            coords={'x', 'y', 'z', 'rx', 'ry', 'rz'};
            if (mod(t, time_plot)==0)
                for coord=1:6
                    subplot(6,1,coord);
                    hold on;
                    plot(t, cartesian_position(:,coord), [IiwaPlotter.ColorCommanded, '.']);
                    if (coord==6)
                        xlabel('time(s)');
                    end
                    ylabel(coords{coord});
                    grid on;
                end
            end
        end
        function cartesian_positions (traj)
            figure;
            coords={'x', 'y', 'z', 'rx', 'ry', 'rz'};
            for coord=1:6
                subplot(6,1,coord);
                hold on;
                plot(traj.t, traj.x(:,coord), IiwaPlotter.ColorCommanded);
                if (coord==6)
                    xlabel('time(s)');
                end
                ylabel(coords{coord});
                grid on;
            end
        end
        function joint_positions(traj)
            figure;
            for j = 1:7
                subplot(7,1,j);
                hold on
                plot(traj.t, traj.q(:,j), IiwaPlotter.ColorCommanded); 
                if j == 1
                    legend('Joint position (rad)','Output joint position');
                    title('Joint positions')
                end
                if j == 7
                    xlabel('time (s)');
                end
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
        end
        function cartesian_positions_compare(traj_comm, traj_output)
            figure;
            coords={'x', 'y', 'z', 'rx', 'ry', 'rz'};
            for coord=1:6
                subplot(6,1,coord);
                hold on;
                plot(traj_comm.t, traj_comm.x(:,coord), IiwaPlotter.ColorCommanded);
                hold on;
                plot(traj_output.t, traj_output.x(:,coord), IiwaPlotter.ColorOutput);
                if coord == 1
                    legend('Commanded cartesian position (rad)','Output cartesian position');
                    title('Commanded and output joint positions')
                end
                if (coord==6)
                    xlabel('time(s)');
                end
                ylabel(coords{coord});
                grid on;
            end
            
        end
        function joint_positions_compare(traj_comm, traj_output)
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
        function joint_positions_compare_big(traj_comm, traj_output)
            for j = 1:7
                figure;
                plot(traj_comm.t, traj_comm.q(:,j), IiwaPlotter.ColorCommanded); 
                hold on
                plot(traj_output.t, traj_output.q(:,j), IiwaPlotter.ColorOutput); 
                legend('Commanded joint position (rad)','Output joint position');
                title('Commanded and output joint positions')
                xlabel('time (s)');
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
           
        end
        function joint_efforts_compare(traj_comm, traj_output)
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
        function joint_effort_error(traj_comm, traj_output)
            figure;
            ts_output = timeseries(traj_comm.effort, traj_comm.t);
            ts_comm = timeseries(traj_output.effort, traj_output.t);
            [ts_output, ts_comm] = synchronize(ts_output, ts_comm, 'Union');

            ts_error = ts_comm-ts_output;
                
            for j=1:7

                subplot(7,1,j);
                plot(ts_error.Time, ts_error.Data(:,j), IiwaPlotter.ColorErrors);
                if j == 1
                    legend('Error joint effort(N)');
                    title('Error between commanded and output efforts')
                end
                if j == 7
                    xlabel('time (s)');
                end
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
        end
        function joint_position_error(traj_comm, traj_output)
            figure;
            ts_output = timeseries(traj_comm.q, traj_comm.t);
            ts_comm = timeseries(traj_output.q, traj_output.t);
            [ts_output, ts_comm] = synchronize(ts_output, ts_comm, 'Union');

            ts_error = ts_comm-ts_output;
                
            for j=1:7

                subplot(7,1,j);
                plot(ts_error.Time, ts_error.Data(:,j), IiwaPlotter.ColorErrors);
                if j == 1
                    legend('Error joint position(rad)');
                    title('Error between commanded and output position')
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

