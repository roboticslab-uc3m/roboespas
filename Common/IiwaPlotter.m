classdef IiwaPlotter < handle
    properties (Constant)
        Colors = {'g', 'b', 'r', 'm'}
        ColorDesired='g'
        ColorCommanded='b'
        ColorOutput='r'
        ColorOthers='m'
        ColorErrors='r'
        TimePlot=0.05;
        plot_points=1;
        plot_limits_qdot=1;
        plot_limits_q=0; %Not implemented
    end
    
    methods
        function obj = IiwaPlotter()
        end
    end
    methods(Static)
        function time_stamps(trajectories, colors)
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            figure;
            leg={};
            for j = 1:IiwaRobot.n_joints
                subplot(IiwaRobot.n_joints,1,j);
                for ntraj=1:size(trajectories,2)
                    if (~isempty(trajectories{ntraj}.q))
                        stamps = trajectories{ntraj}.t(2:end)-trajectories{ntraj}.t(1:end-1);
                        plot(trajectories{ntraj}.t(1:end-1), stamps, [colors(ntraj), '.']);
                        hold on;
                        leg=[leg trajectories{ntraj}.name];
                    end
                end
                if j == 1
                    legend(leg);
                    title('Time stamp difference (second)')
                end
                if j == IiwaRobot.n_joints
                    xlabel('time (s)');
                end
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
        end
        function joint_positions(trajectories, colors)
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            figure;
            leg={};
            for j = 1:IiwaRobot.n_joints
                subplot(IiwaRobot.n_joints,1,j);
                for ntraj=1:size(trajectories,2)
                    if (~isempty(trajectories{ntraj}.q))
                        plot(trajectories{ntraj}.t, rad2deg(trajectories{ntraj}.q(:,j)), colors(ntraj));
                        hold on;
                        leg=[leg trajectories{ntraj}.name];
                        if (IiwaPlotter.plot_points)
                            plot(trajectories{ntraj}.t, rad2deg(trajectories{ntraj}.q(:,j)), [colors(ntraj), '.']);
                            leg=[leg trajectories{ntraj}.name];
                        end
                    end
                end
                if j == 1
                    legend(leg);
                    title('Joint position (rad)')
                end
                if j == IiwaRobot.n_joints
                    xlabel('time (s)');
                end
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
        end
        function joint_velocities(trajectories, colors)
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            figure;
            leg={};
            for j = 1:IiwaRobot.n_joints
                subplot(IiwaRobot.n_joints,1,j);
                for ntraj=1:size(trajectories,2)
                    if (~isempty(trajectories{ntraj}.qdot))
                        plot(trajectories{ntraj}.t, rad2deg(trajectories{ntraj}.qdot(:,j)), [colors(ntraj)]);
                        hold on;
                        leg=[leg trajectories{ntraj}.name];
                        if (IiwaPlotter.plot_points)
                            plot(trajectories{ntraj}.t, rad2deg(trajectories{ntraj}.qdot(:,j)), [colors(ntraj), '.']);
                            leg=[leg trajectories{ntraj}.name];
                        end
                    end
                end
                if j == 1
                    legend(leg);
                    title('Joint velocity (rad/s)')
                end
                if j == IiwaRobot.n_joints
                    xlabel('time (s)');
                end
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
        end
        function joint_accelerations(trajectories, colors)
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            figure;
            leg={};
            for j = 1:IiwaRobot.n_joints
                subplot(IiwaRobot.n_joints,1,j);
                for ntraj=1:size(trajectories,2)
                    if (~isempty(trajectories{ntraj}.qdotdot))
                        plot(trajectories{ntraj}.t, trajectories{ntraj}.qdotdot(:,j), [colors(ntraj)]);
                        hold on;
                        leg=[leg trajectories{ntraj}.name];
                    end
                end
                if j == 1
                    legend(leg);
                    title('Joint acceleration (rad/s2)')
                end
                if j == IiwaRobot.n_joints
                    xlabel('time (s)');
                end
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
        end
        function joint_efforts(trajectories, colors)
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            figure;
            leg={};
            for j = 1:IiwaRobot.n_joints
                subplot(IiwaRobot.n_joints,1,j);
                for ntraj=1:size(trajectories,2)
                    if (~isempty(trajectories{ntraj}.qdot))
                        plot(trajectories{ntraj}.t, trajectories{ntraj}.effort(:,j), [colors(ntraj)]);
                        hold on;
                        leg=[leg trajectories{ntraj}.name];
                    end
                end
                if j == 1
                    legend(leg);
                    title('Joint effort (Nm)')
                end
                if j == IiwaRobot.n_joints
                    xlabel('time (s)');
                end
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
        end
        function joint_efforts_mat(efforts, t, color)
            for j = 1:IiwaRobot.n_joints
                subplot(IiwaRobot.n_joints,1,j);
                plot(t, efforts(:,j), color);
                hold on;
                if j == 1
                    title('Joint effort (Nm)')
                end
                if j == IiwaRobot.n_joints
                    xlabel('time (s)');
                end
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
        end
        function joint_efforts_compare(traj_comm, traj_output)
            figure;
            for j=1:IiwaRobot.n_joints
                subplot(7,1,j);
                plot(traj_comm.t, traj_comm.effort(:,j), IiwaPlotter.ColorCommanded);
                hold on;
                plot(traj_output.t, traj_output.effort(:,j), IiwaPlotter.ColorOutput);
                if j == 1
                    legend('Commanded joint effort(N)','Output joint effort');
                    title('Commanded and output efforts')
                end
                if j == IiwaRobot.n_joints
                    xlabel('time (s)');
                end
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
        end
        
        function cartesian_positions(trajectories, colors)
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            
            figure;
            coords={'x', 'y', 'z', 'rx', 'ry', 'rz'};
            leg={};
            for coord=1:6
                subplot(6,1,coord);
                hold on;
                for ntraj=1:size(trajectories,2)
                    if (~isempty(trajectories{ntraj}.x))
                        plot(trajectories{ntraj}.t, trajectories{ntraj}.x(:,coord),colors(ntraj));
                        plot(trajectories{ntraj}.t, trajectories{ntraj}.x(:,coord),[colors(ntraj), '.']);
                        hold on;
                        leg=[leg trajectories{ntraj}.name trajectories{ntraj}.name];
                    end
                end
                if coord == 1
                    legend(leg);
                    title('Cartesian position (m, rad)')
                end
                if (coord==6)
                    xlabel('time(s)');
                end
                ylabel(coords{coord});
                grid on;
            end
        end
        function cartesian_frames(trajectories, colors, n_frames)
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (~isempty(trajectories{1}.x))
                figure;
                leg=cell(1, size(trajectories,2));
                for ntraj = 1:size(trajectories,2)
                    subsample = round(size(trajectories{ntraj}.x,1)/n_frames);
                    for i =1:subsample:size(trajectories{ntraj}.x,1)
                        IiwaPlotter.frame(trajectories{ntraj}.x(i,:), colors(ntraj));
                        hold on;
                        leg{ntraj} = trajectories{ntraj}.name;
                    end
                end
                legend(leg);
                title('3D cartesian position');
                xlabel('x');
                ylabel('y');
                zlabel('z');
                grid on;
            end  
        end
        function cartesian_velocities(trajectories, colors)
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            
            figure;
            coords={'x', 'y', 'z', 'rx', 'ry', 'rz'};
            leg={};
            for coord=1:6
                subplot(6,1,coord);
                hold on;
                for ntraj=1:size(trajectories,2)
                    if (~isempty(trajectories{ntraj}.x))
                        plot(trajectories{ntraj}.t, trajectories{ntraj}.xdot(:,coord), colors(ntraj));
                        hold on;
                        leg=[leg trajectories{ntraj}.name];
                    end
                end
                if coord == 1
                    legend(leg);
                    title('Cartesian velocity (m/s)')
                end
                if (coord==6)
                    xlabel('time(s)');
                end
                ylabel(coords{coord});
                grid on;
            end
        end
        %% PLOT ERRORS
        function [delays, trajectory_out] = fix_delay_q(traj_baseline, trajectory)
             %Fix delay
            ts_baseline = timeseries(traj_baseline.q, traj_baseline.t);
            ts_traj = timeseries(trajectory.q, trajectory.t);
            mean(traj_baseline.t(2:end)-traj_baseline.t(1:end-1))
            [ts_baseline, ts_traj] =synchronize(ts_baseline, ts_traj, 'Uniform', 'Interval', mean(traj_baseline.t(2:end)-traj_baseline.t(1:end-1)));
            
            delays = finddelay(ts_baseline.Data, ts_traj.Data);
            elem_delete = min(delays);
            trajectory_out = IiwaTrajectory(trajectory.name, trajectory.npoints-elem_delete);
            for j=1:IiwaRobot.n_joints
                d=delays(j);
                if (d~=0)
                    trajectory_out.q(1:trajectory.npoints-d,j) = trajectory.q(1+d:end, j);
                    trajectory_out.q(trajectory.npoints-d+1:end,j) = ones(d-elem_delete,1)*trajectory.q(end-d,j);
                else
                    trajectory_out.q(:,j)=trajectory.q(:,j);
                end
            end
            trajectory_out.t=trajectory.t(1:end-elem_delete);
        end
        function joint_position_error(traj_baseline, trajectories, colors)
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (~isempty(traj_baseline.q))
                figure;
                leg=cell(1, size(trajectories,2));
                ts_baseline = timeseries(traj_baseline.q, traj_baseline.t);
                for ntraj = 1:size(trajectories,2)
                    [delays, trajectories{ntraj}]=IiwaPlotter.fix_delay_q(traj_baseline, trajectories{ntraj});
                    ts_trajectory = timeseries(trajectories{ntraj}.q, trajectories{ntraj}.t);
                    [ts_baseline_used, ts_trajectory_used] = synchronize(ts_baseline, ts_trajectory, 'Intersection');
                    if (size(ts_baseline_used.Time,1)<100)
                        [ts_baseline_used, ts_trajectory_used] = synchronize(ts_baseline, ts_trajectory, 'Union');
                    end
                    ts_error = ts_trajectory_used - ts_baseline_used;
                    for j = 1:IiwaRobot.n_joints
                        subplot(IiwaRobot.n_joints,1,j);
                        plot(ts_error.Time, ts_error.Data(:,j), colors(ntraj));
                        leg{ntraj*2-1} = trajectories{ntraj}.name;
                        hold on;
                        plot(ts_error.Time, ts_error.Data(:,j), [colors(ntraj), '.']);
                        leg{ntraj*2} = trajectories{ntraj}.name;
                        legend(['d = ', num2str(delays(j)*(traj_baseline.t(2)-traj_baseline.t(1)))]);
                        if j == 1
                            title('Joint position error (rad)')
                        end
                        if j == IiwaRobot.n_joints
                            xlabel('time (s)');
                        end
                        s = sprintf('j%d',j);
                        ylabel(s);
                        grid on;
                    end
                end
                grid on;
            end
        end
        function cartesian_position_error(traj_baseline, trajectories, colors)
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            coords={'x', 'y', 'z', 'rx', 'ry', 'rz'};
            if (~isempty(traj_baseline.x))
                figure;
                leg=cell(1, size(trajectories,2));
                ts_baseline = timeseries(traj_baseline.x, traj_baseline.t);
                for ntraj = 1:size(trajectories,2)
                    ts_trajectory = timeseries(trajectories{ntraj}.x, trajectories{ntraj}.t);
                    [ts_baseline_used, ts_trajectory_used] = synchronize(ts_baseline, ts_trajectory, 'Intersection');
                    if (size(ts_baseline_used.Time,1)<100)
                        [ts_baseline_used, ts_trajectory_used] = synchronize(ts_baseline, ts_trajectory, 'Union');
                    end
                    error = zeros(size(ts_trajectory_used.Data,1), 6);
                    for i=1:size(ts_trajectory_used.Data,1)
                        error(i,:) = IiwaScrewTheory.screwA2B_A(ts_baseline_used.Data(i,:), ts_trajectory_used.Data(i,:));
                    end
                    ts_error = timeseries(error, ts_baseline_used.Time);
                    for coord = 1:6
                        subplot(6,1,coord);
                        plot(ts_error.Time, ts_error.Data(:,coord), colors(ntraj));
                        leg{ntraj*2-1} = trajectories{ntraj}.name;
                        hold on;
                        plot(ts_error.Time, ts_error.Data(:,coord), [colors(ntraj), '.']);
                        leg{ntraj*2} = trajectories{ntraj}.name;
                        if coord == 1
                            title('Cartesian position error (m, rad)')
                        end
                        if coord == 6
                            xlabel('time (s)');
                        end
                        ylabel(coords{coord});
                        grid on;
                    end
                end
                legend(leg);
                grid on;
            end
        end
        function effortWithPD(traj_des, traj_comm)
            figure;
            ts_des=timeseries(traj_comm.effort, traj_comm.t);
            ts_comm=timeseries(traj_des.effort, traj_des.t);
            [ts_des, ts_comm] = synchronize(ts_des, ts_comm, 'Intersection');
            if (size(ts_des.Time,1)<100)
            [ts_des, ts_comm] = synchronize(ts_des, ts_comm, 'Union');
            end
            ts_pdTorque=ts_comm-ts_des;
            for j = 1:IiwaRobot.n_joints
                subplot(IiwaRobot.n_joints,1,j);
                plot(traj_des.t, traj_des.effort(:,j), IiwaPlotter.ColorDesired); hold on
                plot(ts_pdTorque.Time, ts_pdTorque.Data(:,j), IiwaPlotter.ColorOthers);
                plot(traj_comm.t, traj_comm.effort(:,j), IiwaPlotter.ColorCommanded);
                if j == 1
                    legend('Torque desired (N)','PD torque','Total torque');
                    title('Commanded and output efforts with PD efforts')
                end
                if j == IiwaRobot.n_joints
                    xlabel('time (s)');
                end
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
        end
%         function joint_position_error(traj_comm, traj_output)
%             figure;
%             ts_output = timeseries(traj_comm.q, traj_comm.t);
%             ts_comm = timeseries(traj_output.q, traj_output.t);
%             [ts_output, ts_comm] = synchronize(ts_output, ts_comm, 'Union');
% 
%             ts_error = ts_comm-ts_output;
%                 
%             for j=1:7
% 
%                 subplot(7,1,j);
%                 plot(ts_error.Time, ts_error.Data(:,j), IiwaPlotter.ColorErrors);
%                 if j == 1
%                     legend('Error joint position(rad)');
%                     title('Error between commanded and output position')
%                 end
%                 if j == 7
%                     xlabel('time (s)');
%                 end
%                 s = sprintf('j%d',j);
%                 ylabel(s);
%                 grid on;
%             end
%         end
        %% INDIVIDUAL PLOTS
        function frame(frame_given, color)
            if (size(frame_given)==[6, 1])
                frame_given=frame_given';
            end
            origin=frame_given(1:3);
            rotation=frame_given(4:6);
            R=eul2rotm(rotation, 'XYZ');
            quiver3(origin(1), origin(2), origin(3), R(1,1), R(2,1), R(3,1), 0.1, [color, '-']);
            hold on;
            quiver3(origin(1), origin(2), origin(3), R(1,2), R(2,2), R(3,2), 0.1, [color, '--']);
            quiver3(origin(1), origin(2), origin(3), R(1,3), R(2,3), R(3,3), 0.1, [color, '-.']);

            xlabel('x')
            ylabel('y')
            zlabel('z')
        end
        function joint_position(joint_position, t)
            if (mod(t, IiwaPlotter.TimePlot)==0)
                for j = 1:IiwaRobot.n_joints
                    subplot(IiwaRobot.n_joints,1,j);
                    hold on
                    plot(t, joint_position(:,j), [IiwaPlotter.ColorCommanded, '.']); 
                    if j == 1
                        title(['Joint positions from 0 to ', num2str(t), ' (rad)'])
                    end
                    if j == IiwaRobot.n_joints
                        xlabel('time (s)');
                    end
                    s = sprintf('j%d',j);
                    ylabel(s);
                    grid on;
                end
            end
        end
        function cartesian_position(cartesian_position, t)
            coords={'x', 'y', 'z', 'rx', 'ry', 'rz'};
            if (mod(t, IiwaPlotter.TimePlot)==0)
                for coord=1:6
                    subplot(6,1,coord);
                    hold on;
                    plot(t, cartesian_position(:,coord), [IiwaPlotter.ColorCommanded, '.']);
                    if coord == 1
                        title(['Cartesian positions from 0 to ', num2str(t), ' (m, rad)'])
                    end
                    if (coord==6)
                        xlabel('time(s)');
                    end
                    ylabel(coords{coord});
                    grid on;
                end
            end
        end
        function joint_effort(effort_commanded, effort_state, t)
           % Time between points plotted. You may need to change this depending on 
           % your computer's resources
           if (mod(t, IiwaPlotter.TimePlot)==0)
                for j=1:IiwaRobot.n_joints
                    subplot(IiwaRobot.n_joints,1,j);
                    if j == 1
                        title(['Joint efforts from 0 to ', num2str(t), ' (rad)'])
                    end
                    if j == IiwaRobot.n_joints
                        xlabel('time (s)');
                    end
                    s = sprintf('e%d',j);
                    ylabel(s);
                    plot(t, effort_commanded.Effort(j), ['.', IiwaPlotter.ColorCommanded]);
                    hold on;
                    plot(t, effort_state(j), ['.', IiwaPlotter.ColorOutput]);
                end
           end
        end
        %% TODO: Change to trajectories cell
        function effortWithPD_compare_big(traj_comm, traj_output, traj_withoutPD)
            for i=1:IiwaRobot.n_joints
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

        function joint_effort_error(traj_comm, traj_output)
            figure;
            ts_output = timeseries(traj_comm.effort, traj_comm.t);
            ts_comm = timeseries(traj_output.effort, traj_output.t);
            [ts_output, ts_comm] = synchronize(ts_output, ts_comm, 'Union');

            ts_error = ts_comm-ts_output;
                
            for j=1:IiwaRobot.n_joints

                subplot(IiwaRobot.n_joints,1,j);
                plot(ts_error.Time, ts_error.Data(:,j), IiwaPlotter.ColorErrors);
                if j == 1
                    legend('Error joint effort(N)');
                    title('Error between commanded and output efforts')
                end
                if j == IiwaRobot.n_joints
                    xlabel('time (s)');
                end
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
        end
        function joint_positions_compare_big(traj_comm, traj_output)
            for j = 1:IiwaRobot.n_joints
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
    end
end

