classdef IiwaPlotter < handle
    properties (Constant)
        Colors = {'g', 'b', 'r', 'm'}
        ColorDesired='g'
        ColorCommanded='b'
        ColorOutput='r'
        ColorOthers='m'
        ColorErrors='r'
        TimePlot=0.05;
        plot_points=0;
        plot_limits_qdot=1;
        plot_limits_q=0; %Not implemented
        LineWidth = 2;
        ColorsLightNames=['r', 'b'];
        ColorsLight=[[243/256, 163/256, 152/256]; [152/256, 163/256, 243/256]];
        ColorsXYZ = ['b', 'r', 'g'];
        CoordNames = ['X', 'Y', 'Z', 'A', 'B', 'C']
        CoordUnits = {'cm', 'cm', 'cm', 'deg', 'deg', 'deg'}
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
                    xlabel('Time [s]');
                end
                s = sprintf('J%d',j);
                ylabel(s);
                grid on;
            end
        end
        function joint_positions(trajectories, colors, varargin)
            s = [1,3,5,7,2,4,6]; %Subplot order
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (isempty(varargin))
                figure;
            else
                display = varargin{1};
            end
            leg={};
            ax = zeros(1,7);
            for j = 1:IiwaRobot.n_joints
                if (isempty(varargin))
                    ax(j) = subplot(4,2,s(j));
                else
                    ax(j) = subplot(4,2,s(j), 'Parent', display);
                end
                for ntraj=1:size(trajectories,2)
                    if (~isempty(trajectories{ntraj}.q))
                        plot(ax(j), trajectories{ntraj}.t, rad2deg(trajectories{ntraj}.q(:,j)), colors(ntraj), 'LineWidth', IiwaPlotter.LineWidth);
                        hold(ax(j), 'on');
                        leg=[leg trajectories{ntraj}.name];
                        if (IiwaPlotter.plot_points)
                            plot(ax(j), trajectories{ntraj}.t, rad2deg(trajectories{ntraj}.q(:,j)), [colors(ntraj), '.']);
                            leg=[leg trajectories{ntraj}.name];
                        end
                    end
                end
                if (j == 7)
                	legend(ax(j), leg);
                end
                if (j == 1 && isempty(varargin))
                    title(ax(j), 'Joint position (º)')
                end
                if (j == IiwaRobot.n_joints || ~isempty(varargin))
                    xlabel(ax(j), 'Time [s]');
                end
                title(ax(j), ['J', num2str(j)])
                ylabel(ax(j), '[deg]');
                grid(ax(j),'on');
            end
        end
        function joint_velocities(trajectories, colors, varargin)
            s = [1,3,5,7,2,4,6]; %Subplot order
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (isempty(varargin))
                figure;
            else
                display = varargin{1};
            end
            leg={};
            ax = zeros(1,7);
            for j = 1:IiwaRobot.n_joints
                if (isempty(varargin))
                    ax(j) = subplot(4,2,s(j));
                else
                    ax(j) = subplot(4,2,s(j), 'Parent', display);
                end
                for ntraj=1:size(trajectories,2)
                    if (~isempty(trajectories{ntraj}.qdot))
                        plot(ax(j), trajectories{ntraj}.t, rad2deg(trajectories{ntraj}.qdot(:,j)), [colors(ntraj)], 'LineWidth', IiwaPlotter.LineWidth);
                        hold(ax(j), 'on');
                        leg=[leg trajectories{ntraj}.name];
                        if (IiwaPlotter.plot_points)
                            plot(ax(j), trajectories{ntraj}.t, rad2deg(trajectories{ntraj}.qdot(:,j)), [colors(ntraj), '.']);
                            leg=[leg trajectories{ntraj}.name];
                        end
                    end
                end
                if (j == 7 && isempty(varargin))
                    title(ax(j), 'Joint velocity (rad/s)')
                end
                if (j == IiwaRobot.n_joints || ~isempty(varargin))
                    xlabel(ax(j), 'Time [s]');
                end
                if j == 7
                    legend(ax(j), leg);
                end
                title(ax(j), ['J', num2str(j)])
                ylabel(ax(j), '[deg/s]');
                grid(ax(j), 'on')
            end
        end
        function joint_accelerations(trajectories, colors, varargin)
            s = [1,3,5,7,2,4,6]; %Subplot order
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (isempty(varargin))
                figure;
            else
                display = varargin{1};
            end
            leg={};
            ax = zeros(1,7);
            for j = 1:IiwaRobot.n_joints
                if (isempty(varargin))
                    ax(j) = subplot(4,2,s(j));
                else
                    ax(j) = subplot(4,2,s(j), 'Parent', display);
                end
                for ntraj=1:size(trajectories,2)
                    if (~isempty(trajectories{ntraj}.qdotdot))
                        plot(ax(j), trajectories{ntraj}.t, rad2deg(trajectories{ntraj}.qdotdot(:,j)), [colors(ntraj)], 'LineWidth', IiwaPlotter.LineWidth);
                        hold(ax(j), 'on');
                        leg=[leg trajectories{ntraj}.name];
                        if (IiwaPlotter.plot_points)
                            plot(ax(j), trajectories{ntraj}.t, rad2deg(trajectories{ntraj}.qdotdot(:,j)), [colors(ntraj), '.']);
                            leg=[leg trajectories{ntraj}.name];
                        end
                    end
                end
                if (j == 1 && isempty(varargin))
                    title(ax(j), 'Joint acceleration (rad/s2)')
                end
                if (j == IiwaRobot.n_joints || ~isempty(varargin))
                    xlabel(ax(j), 'Time [s]');
                end
                if j == 7
                    legend(ax(j), leg);
                end
                title(ax(j), ['J', num2str(j)])
                ylabel(ax(j), '[deg/s2]');
                grid(ax(j), 'on')
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
                    xlabel('time [s]');
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
                    xlabel('time [s]');
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
                    xlabel('time [s]');
                end
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
        end      
        function cartesian_positions(trajectories, colors, varargin)
            s = [1,3,5,2,4,6]; %Subplot order
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (isempty(varargin))
                figure;
            else
                display = varargin{1};
            end
            leg={};
            ax = zeros(1,7);
            for coord = 1:3
                if (isempty(varargin))
                    ax(coord) = subplot(3,2,s(coord));
                else
                    ax(coord) = subplot(3,2,s(coord), 'Parent', display);
                end
                for ntraj=1:size(trajectories,2)
                    if (~isempty(trajectories{ntraj}.x))
                        plot(ax(coord), trajectories{ntraj}.t, 100*(trajectories{ntraj}.x(:,coord)), colors(ntraj), 'LineWidth', IiwaPlotter.LineWidth);
                        hold(ax(coord), 'on');
                        leg=[leg trajectories{ntraj}.name];
                        if (IiwaPlotter.plot_points)
                            plot(ax(coord), trajectories{ntraj}.t, 100*(trajectories{ntraj}.x(:,coord)), [colors(ntraj), '.']);
                            leg=[leg trajectories{ntraj}.name];
                        end
                    end
                end
                xlabel(ax(coord), 'Time [s]');
                title(ax(coord), IiwaPlotter.CoordNames(coord));
                ylabel(ax(coord), ['[', IiwaPlotter.CoordUnits{coord}, ']']);
                grid(ax(coord),'on');
            end
           for coord = 4:6
                if (isempty(varargin))
                    ax(coord) = subplot(3,2,s(coord));
                else
                    ax(coord) = subplot(3,2,s(coord), 'Parent', display);
                end
                for ntraj=1:size(trajectories,2)
                    if (~isempty(trajectories{ntraj}.x))
                        plot(ax(coord), trajectories{ntraj}.t, rad2deg(trajectories{ntraj}.x(:,coord)), colors(ntraj), 'LineWidth', IiwaPlotter.LineWidth);
                        hold(ax(coord), 'on');
                        leg=[leg trajectories{ntraj}.name];
                        if (IiwaPlotter.plot_points)
                            plot(ax(coord), trajectories{ntraj}.t, rad2deg(trajectories{ntraj}.x(:,coord)), [colors(ntraj), '.']);
                            leg=[leg trajectories{ntraj}.name];
                        end
                    end
                end
                if (coord == 6)
                	legend(ax(coord), leg{1:size(trajectories,2)});
                end
                xlabel(ax(coord), 'Time [s]');
                title(ax(coord), IiwaPlotter.CoordNames(coord));
                ylabel(ax(coord), ['[', IiwaPlotter.CoordUnits{coord}, ']']);
                grid(ax(coord),'on');
            end
        end
        function cartesian_frames(trajectories, colors, n_frames, varargin)
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
                axis equal;
                grid on;
            end  
        end
        function cartesian_positions3d(trajectories, colors, varargin)
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (~isempty(trajectories{1}.x))
                if (isempty(varargin))
                    figure;
                else
                    display = varargin{1};
                end
                leg={}
                for ntraj = 1:size(trajectories,2)
                    if (~isempty(varargin))
                        ax = subplot(1,1,1,'Parent', display);
                    else
                        ax = subplot(1,1,1);
                    end
                    if (ntraj==1)
                        hold(ax, 'off')
                    end
                    plot3(ax, trajectories{ntraj}.x(:,1), trajectories{ntraj}.x(:,2), trajectories{ntraj}.x(:,3), char(colors(ntraj)), 'LineWidth', IiwaPlotter.LineWidth);
                    hold(ax, 'on');
                    leg=[leg trajectories{ntraj}.name];
                end
                legend(ax,leg, 'Location', 'southeast');
                if (isempty(varargin))
                    title(ax, '3D cartesian position');
                end
                xlabel(ax, 'x');
                ylabel(ax, 'y');
                zlabel(ax, 'z');
                grid(ax, 'on');
                view(ax, [-120, 30]);
                axis(ax, 'equal');
            end 
        end
        function cartesian_velocities(trajectories, colors, varargin)
            s = [1,3,5,2,4,6]; %Subplot order
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (isempty(varargin))
                figure;
            else
                display = varargin{1};
            end
            leg={};
            ax = zeros(1,7);
            subsample=1;
            for coord = 1:3
                if (isempty(varargin))
                    ax(coord) = subplot(3,2,s(coord));
                else
                    ax(coord) = subplot(3,2,s(coord), 'Parent', display);
                end
                for ntraj=1:size(trajectories,2)
                    if (~isempty(trajectories{ntraj}.xdot))
                        plot(ax(coord), trajectories{ntraj}.t(1:subsample:end), 100*(trajectories{ntraj}.xdot(1:subsample:end,coord)), colors(ntraj), 'LineWidth', IiwaPlotter.LineWidth);
                        hold(ax(coord), 'on');
                        leg=[leg trajectories{ntraj}.name];
                        if (IiwaPlotter.plot_points)
                            plot(ax(coord), trajectories{ntraj}.t(1:subsample:end), 100*(trajectories{ntraj}.xdot(1:subsample:end,coord)), [colors(ntraj), '.']);
                            leg=[leg trajectories{ntraj}.name];
                        end
                    end
                end
                if (coord == 6)
                	legend(ax(coord), leg{1:size(trajectories,2)});
                end
                xlabel(ax(coord), 'Time [s]');
                title(ax(coord), IiwaPlotter.CoordNames(coord));
                ylabel(ax(coord), ['[', IiwaPlotter.CoordUnits{coord}, '/s]']);
                grid(ax(coord),'on');
            end
            for coord = 4:6
                if (isempty(varargin))
                    ax(coord) = subplot(3,2,s(coord));
                else
                    ax(coord) = subplot(3,2,s(coord), 'Parent', display);
                end
                for ntraj=1:size(trajectories,2)
                    if (~isempty(trajectories{ntraj}.xdot))
                        plot(ax(coord), trajectories{ntraj}.t(1:subsample:end), rad2deg(trajectories{ntraj}.xdot(1:subsample:end,coord)), colors(ntraj), 'LineWidth', IiwaPlotter.LineWidth);
                        hold(ax(coord), 'on');
                        leg=[leg trajectories{ntraj}.name];
                        if (IiwaPlotter.plot_points)
                            plot(ax(coord), trajectories{ntraj}.t(1:subsample:end), rad2deg(trajectories{ntraj}.xdot(1:subsample:end,coord)), [colors(ntraj), '.']);
                            leg=[leg trajectories{ntraj}.name];
                        end
                    end
                end
                if (coord == 6)
                	legend(ax(coord), leg{1:size(trajectories,2)});
                end
                xlabel(ax(coord), 'Time [s]');
                title(ax(coord), IiwaPlotter.CoordNames(coord));
                ylabel(ax(coord), ['[', IiwaPlotter.CoordUnits{coord}, '/s]']);
                grid(ax(coord),'on');
            end
        end
        function fill_joint_position(traj_1, traj_2, color, varargin)
            s = [1,3,5,7,2,4,6]; %subplot order
            ax = zeros(1,7);
            for j = 1:7
                if (isempty(varargin))
                    ax(j) = subplot(4,2,s(j));
                else
                    display=varargin{1};
                    ax(j) = subplot(4,2,s(j), 'Parent', display);
                end
                hold(ax(j), 'on');
                x=[traj_1.t; flipud(traj_2.t)];
                y=[rad2deg(traj_1.q(:,j)); flipud(rad2deg(traj_2.q(:,j)))];
                fill(ax(j), x,y, IiwaPlotter.ColorsLight(find(color==IiwaPlotter.ColorsLightNames),:));
            end
        end
        function fill_joint_velocity(traj_1, traj_2, color, varargin)
            s = [1,3,5,7,2,4,6]; %subplot order
            ax = zeros(1,7);
            for j = 1:7
                if (isempty(varargin))
                    ax(j) = subplot(4,2,s(j));
                else
                    display=varargin{1};
                    ax(j) = subplot(4,2,s(j), 'Parent', display);
                end
                hold(ax(j), 'on');
                x=[traj_1.t; flipud(traj_2.t)];
                y=[rad2deg(traj_1.qdot(:,j)); flipud(rad2deg(traj_2.qdot(:,j)))];
                fill(ax(j), x,y, IiwaPlotter.ColorsLight(color==IiwaPlotter.ColorsLightNames,:));
            end
        end
        function fill_joint_acceleration(traj_1, traj_2, color, varargin)
            s = [1,3,5,7,2,4,6]; %subplot order
            ax = zeros(1,7);
            for j = 1:7
                if (isempty(varargin))
                    ax(j) = subplot(4,2,s(j));
                else
                    display=varargin{1};
                    ax(j) = subplot(4,2,s(j), 'Parent', display);
                end
                x=[traj_1.t; flipud(traj_2.t)];
                y=[rad2deg(traj_1.qdotdot(:,j)); flipud(rad2deg(traj_2.qdotdot(:,j)))];
                fill(ax(j), x,y, IiwaPlotter.ColorsLight(color==IiwaPlotter.ColorsLightNames,:));
                hold(ax(j), 'on');
            end
        end
        function fill_cartesian_position(traj_1, traj_2, color, varargin)
            s = [1,3,5,2,4,6]; %subplot order
            ax = zeros(1,6);
            for coord = 1:3
                if (isempty(varargin))
                    ax(coord) = subplot(3,2,s(coord));
                else
                    display=varargin{1};
                    ax(coord) = subplot(3,2,s(coord), 'Parent', display);
                end
                x=[traj_1.t; flipud(traj_2.t)];
                y=[100*(traj_1.x(:,coord)); flipud(100*(traj_2.x(:,coord)))];
                fill(ax(coord), x,y, IiwaPlotter.ColorsLight(color==IiwaPlotter.ColorsLightNames,:));
                hold(ax(coord), 'on');
            end
            for coord = 4:6
                if (isempty(varargin))
                    ax(coord) = subplot(3,2,s(coord));
                else
                    display=varargin{1};
                    ax(coord) = subplot(3,2,s(coord), 'Parent', display);
                end
                x=[traj_1.t; flipud(traj_2.t)];
                y=[rad2deg(traj_1.x(:,coord)); flipud(rad2deg(traj_2.x(:,coord)))];
                fill(ax(coord), x,y, IiwaPlotter.ColorsLight(color==IiwaPlotter.ColorsLightNames,:));
                hold(ax(coord), 'on');
            end
        end
        function fill_cartesian_velocity(traj_1, traj_2, color, varargin)
            s = [1,3,5,2,4,6]; %subplot order
            ax = zeros(1,6);
            for coord = 1:3
                if (isempty(varargin))
                    ax(coord) = subplot(3,2,s(coord));
                else
                    display=varargin{1};
                    ax(coord) = subplot(3,2,s(coord), 'Parent', display);
                end
                hold(ax(coord), 'on');
                x=[traj_1.t; flipud(traj_2.t)];
                y=[100*(traj_1.xdot(:,coord)); flipud(100*(traj_2.xdot(:,coord)))];
                fill(ax(coord), x,y, IiwaPlotter.ColorsLight(color==IiwaPlotter.ColorsLightNames,:));
            end
            for coord = 4:6
                if (isempty(varargin))
                    ax(coord) = subplot(3,2,s(coord));
                else
                    display=varargin{1};
                    ax(coord) = subplot(3,2,s(coord), 'Parent', display);
                end
                hold(ax(coord), 'on');
                x=[traj_1.t; flipud(traj_2.t)];
                y=[rad2deg(traj_1.xdot(:,coord)); flipud(rad2deg(traj_2.xdot(:,coord)))];
                fill(ax(coord), x,y, IiwaPlotter.ColorsLight(color==IiwaPlotter.ColorsLightNames,:));
            end
        end
        %% PLOT ERRORS
%         function [delays, data_out, t_out] = fix_delay_q(baseline, baseline_t, data, data_t)
%              %Fix delay
%             ts_baseline = timeseries(baseline, baseline_t);
%             ts_traj = timeseries(data, data_t);
%             [ts_baseline, ts_traj] =synchronize(ts_baseline, ts_traj, 'Uniform', 'Interval', mean(baseline_t(2:end)-baseline_t(1:end-1)));
%             
%             delays = finddelay(ts_baseline.Data, ts_traj.Data);
%             elem_delete = min(delays);
%             for j=1:IiwaRobot.n_joints
%                 d=delays(j);
%                 if (d~=0)
%                     trajectory_out.q(1:trajectory.npoints-d,j) = trajectory.q(1+d:end, j);
%                     trajectory_out.q(trajectory.npoints-d+1:end,j) = ones(d-elem_delete,1)*trajectory.q(end-d,j);
%                 else
%                     trajectory_out.q(:,j)=trajectory.q(:,j);
%                 end
%             end
%             trajectory_out.t=trajectory.t(1:end-elem_delete);
%             trajectory_out = trajectory_out.CompleteCartesian();
%             trajectory_out = trajectory_out.CompleteVelAcc();
%         end
        function joint_position_error(traj_baseline, trajectories, colors, varargin)
            s = [1,3,5,7,2,4,6]; %Subplot order
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (isempty(varargin))
                figure;
            else
                display = varargin{1};
            end
            leg={};
            ax = zeros(1,7);
            ts_baseline = timeseries(traj_baseline.q, traj_baseline.t);
            for ntraj = 1:size(trajectories,2)
                ts_trajectory = timeseries(trajectories{ntraj}.q, trajectories{ntraj}.t);
                [ts_baseline_used, ts_trajectory_used] = synchronize(ts_baseline, ts_trajectory, 'Intersection');
                st_baseline = mean(traj_baseline.t(2:end)-traj_baseline.t(1:end-1));
                st_traj = mean(trajectories{ntraj}.t(2:end)-trajectories{ntraj}.t(1:end-1));
                if (size(ts_baseline_used.Time,1)<100)
                    [ts_baseline_used, ts_trajectory_used] = synchronize(ts_baseline, ts_trajectory, 'Uniform', 'interval', (st_baseline+st_traj)*2);
                end
                ts_error = ts_trajectory_used-ts_baseline_used;
                for j = 1:IiwaRobot.n_joints
                    if (isempty(varargin))
                        ax(j) = subplot(4,2,s(j));
                    else
                        ax(j) = subplot(4,2,s(j), 'Parent', display);
                    end
                    rms_error(j) = rad2deg(rms(ts_error.Data(:,j))); 
                    dashedcolor = ['--', colors(ntraj)];
                    plot(ax(j), ts_error.Time', ones(length(ts_error.Time),1).*rms_error(j)', dashedcolor);
                    hold(ax(j), 'on');
                    plot(ax(j), ts_error.Time, rad2deg(ts_error.Data(:,j)), colors(ntraj), 'LineWidth', IiwaPlotter.LineWidth);
                    leg=[leg trajectories{ntraj}.name];
                    if (IiwaPlotter.plot_points)
                        plot(ax(j), ts_error.Time, rad2deg(ts_error.Data(:,j)), [colors(ntraj), '.']);
                        leg=[leg trajectories{ntraj}.name];
                    end
                    legend(ax(j), ['Error_{', trajectories{ntraj}.name, '}'], ['rms_{', trajectories{ntraj}.name, '} = ', num2str(rms_error(j),3), 'deg']);
                    title(ax(j), ['J', num2str(j)])
                    ylabel(ax(j), ['[deg]']);
                    xlabel(ax(j), 'Time [s]');
                    grid(ax(j),'on');
                end
            end
        end
        function joint_position_rms(trajectories, colors, varargin)
            s = [1,3,5,7,2,4,6]; %Subplot order
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (isempty(varargin))
                figure;
            else
                display = varargin{1};
            end
            leg={};
            ax = zeros(1,7);
            for ntraj = 1:size(trajectories,2)
                for j = 1:IiwaRobot.n_joints
                    if (isempty(varargin))
                        ax(j) = subplot(4,2,s(j));
                    else
                        ax(j) = subplot(4,2,s(j), 'Parent', display);
                    end
                    rms_error(j) = rad2deg(rms(trajectories{ntraj}.q(:,j))); 
                    dashedcolor = ['--', colors(ntraj)];
                    plot(ax(j), trajectories{ntraj}.t', ones(length(trajectories{ntraj}.t),1).*rms_error(j)', dashedcolor);
                    legend(ax(j), ['Error_{', trajectories{ntraj}.name, '}'], ['rms_{', trajectories{ntraj}.name, '} = ', num2str(rms_error(j),3), 'deg']);
                    hold(ax(j), 'on');
                end
            end
        end
        function joint_velocity_rms(trajectories, colors, varargin)
            s = [1,3,5,7,2,4,6]; %Subplot order
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (isempty(varargin))
                figure;
            else
                display = varargin{1};
            end
            leg={};
            ax = zeros(1,7);
            for ntraj = 1:size(trajectories,2)
                for j = 1:IiwaRobot.n_joints
                    if (isempty(varargin))
                        ax(j) = subplot(4,2,s(j));
                    else
                        ax(j) = subplot(4,2,s(j), 'Parent', display);
                    end
                    rms_error(j) = rad2deg(rms(trajectories{ntraj}.qdot(:,j))); 
                    dashedcolor = ['--', colors(ntraj)];
                    plot(ax(j), trajectories{ntraj}.t', ones(length(trajectories{ntraj}.t),1).*rms_error(j)', dashedcolor);
                    legend(ax(j), ['Error_{', trajectories{ntraj}.name, '}'], ['rms_{', trajectories{ntraj}.name, '} = ', num2str(rms_error(j),3), 'deg/s']);
                    hold(ax(j), 'on');
                end
            end
        end
        function joint_acceleration_rms(trajectories, colors, varargin)
            s = [1,3,5,7,2,4,6]; %Subplot order
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (isempty(varargin))
                figure;
            else
                display = varargin{1};
            end
            leg={};
            ax = zeros(1,7);
            for ntraj = 1:size(trajectories,2)
                for j = 1:IiwaRobot.n_joints
                    if (isempty(varargin))
                        ax(j) = subplot(4,2,s(j));
                    else
                        ax(j) = subplot(4,2,s(j), 'Parent', display);
                    end
                    rms_error(j) = rad2deg(rms(trajectories{ntraj}.qdotdot(:,j))); 
                    dashedcolor = ['--', colors(ntraj)];
                    plot(ax(j), trajectories{ntraj}.t', ones(length(trajectories{ntraj}.t),1).*rms_error(j)', dashedcolor);
                    legend(ax(j), ['Error_{', trajectories{ntraj}.name, '}'], ['rms_{', trajectories{ntraj}.name, '} = ', num2str(rms_error(j),3), 'deg/s2']);
                    hold(ax(j), 'on');
                end
            end
        end
        function cartesian_position_rms(trajectories, colors, varargin)
            s = [1,3,5,2,4,6]; %Subplot order
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (isempty(varargin))
                figure;
            else
                display = varargin{1};
            end
            leg={};
            ax = zeros(1,7);
            for ntraj = 1:size(trajectories,2)
                for j = 1:3
                    if (isempty(varargin))
                        ax(j) = subplot(3,2,s(j));
                    else
                        ax(j) = subplot(3,2,s(j), 'Parent', display);
                    end
                    rms_error(j) = 100*(rms(trajectories{ntraj}.x(:,j))); 
                    dashedcolor = ['--', colors(ntraj)];
                    plot(ax(j), trajectories{ntraj}.t', ones(length(trajectories{ntraj}.t),1).*rms_error(j)', dashedcolor);
                    legend(ax(j), ['Error_{', trajectories{ntraj}.name, '}'], ['rms_{', trajectories{ntraj}.name, '} = ', num2str(rms_error(j),3), 'cm']);
                    hold(ax(j), 'on');
                end
                for j = 4:6
                    if (isempty(varargin))
                        ax(j) = subplot(3,2,s(j));
                    else
                        ax(j) = subplot(3,2,s(j), 'Parent', display);
                    end
                    rms_error(j) = rad2deg(rms(trajectories{ntraj}.x(:,j))); 
                    dashedcolor = ['--', colors(ntraj)];
                    plot(ax(j), trajectories{ntraj}.t', ones(length(trajectories{ntraj}.t),1).*rms_error(j)', dashedcolor);
                    legend(ax(j), ['Error_{', trajectories{ntraj}.name, '}'], ['rms_{', trajectories{ntraj}.name, '} = ', num2str(rms_error(j),3), 'deg']);
                    hold(ax(j), 'on');
                end
            end
            
        end
        function cartesian_velocity_rms(trajectories, colors, varargin)
            s = [1,3,5,2,4,6]; %Subplot order
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (isempty(varargin))
                figure;
            else
                display = varargin{1};
            end
            leg={};
            ax = zeros(1,7);
            for ntraj = 1:size(trajectories,2)
                for j = 1:3
                    if (isempty(varargin))
                        ax(j) = subplot(3,2,s(j));
                    else
                        ax(j) = subplot(3,2,s(j), 'Parent', display);
                    end
                    rms_error(j) = 100*(rms(trajectories{ntraj}.xdot(:,j))); 
                    dashedcolor = ['--', colors(ntraj)];
                    plot(ax(j), trajectories{ntraj}.t', ones(length(trajectories{ntraj}.t),1).*rms_error(j)', dashedcolor);
                    legend(ax(j), ['Error_{', trajectories{ntraj}.name, '}'], ['rms_{', trajectories{ntraj}.name, '} = ', num2str(rms_error(j),3), 'cm']);
                    hold(ax(j), 'on');
                end
                for j = 4:6
                    if (isempty(varargin))
                        ax(j) = subplot(3,2,s(j));
                    else
                        ax(j) = subplot(3,2,s(j), 'Parent', display);
                    end
                    rms_error(j) = rad2deg(rms(trajectories{ntraj}.xdot(:,j))); 
                    dashedcolor = ['--', colors(ntraj)];
                    plot(ax(j), trajectories{ntraj}.t', ones(length(trajectories{ntraj}.t),1).*rms_error(j)', dashedcolor);
                    legend(ax(j), ['Error_{', trajectories{ntraj}.name, '}'], ['rms_{', trajectories{ntraj}.name, '} = ', num2str(rms_error(j),3), 'deg']);
                    hold(ax(j), 'on');
                end
            end
        end
        function joint_velocity_error(traj_baseline, trajectories, colors, varargin)
            s = [1,3,5,7,2,4,6]; %Subplot order
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (isempty(varargin))
                figure;
            else
                display = varargin{1};
            end
            leg={};
            ax = zeros(1,7);
            ts_baseline = timeseries(traj_baseline.qdot, traj_baseline.t);
            for ntraj = 1:size(trajectories,2)
                ts_trajectory = timeseries(trajectories{ntraj}.qdot, trajectories{ntraj}.t);
                [ts_baseline_used, ts_trajectory_used] = synchronize(ts_baseline, ts_trajectory, 'Intersection');
                st_baseline = mean(traj_baseline.t(2:end)-traj_baseline.t(1:end-1));
                st_traj = mean(trajectories{ntraj}.t(2:end)-trajectories{ntraj}.t(1:end-1));
                if (size(ts_baseline_used.Time,1)<100)
                    [ts_baseline_used, ts_trajectory_used] = synchronize(ts_baseline, ts_trajectory, 'Uniform', 'interval', (st_baseline+st_traj)*2);
                end
                ts_error = ts_trajectory_used-ts_baseline_used;
                for j = 1:IiwaRobot.n_joints
                    if (isempty(varargin))
                        ax(j) = subplot(4,2,s(j));
                    else
                        ax(j) = subplot(4,2,s(j), 'Parent', display);
                    end
                    rms_error(j) = rms(ts_error.Data(:,j)); 
                    dashedcolor = ['--', colors(ntraj)];
                    plot(ax(j), ts_error.Time', ones(length(ts_error.Time),1).*rms_error(j)', dashedcolor);
                    hold(ax(j), 'on');
                    plot(ax(j), ts_error.Time, rad2deg(ts_error.Data(:,j)), colors(ntraj), 'LineWidth', IiwaPlotter.LineWidth);
                    leg=[leg trajectories{ntraj}.name];
                    if (IiwaPlotter.plot_points)
                        plot(ax(j), ts_error.Time, rad2deg(ts_error.Data(:,j)), [colors(ntraj), '.']);
                        leg=[leg trajectories{ntraj}.name];
                    end
                    legend(ax(j), ['Error_{', trajectories{ntraj}.name, '}'], ['rms_{', trajectories{ntraj}.name, '} = ', num2str(rms_error(j),3), 'deg/s']);
                    title(ax(j), ['J', num2str(j)])
                    ylabel(ax(j), '[deg/s]');
                    xlabel(ax(j), 'Time [s]');
                    grid(ax(j),'on');
                end
            end
        end
        function joint_acceleration_error(traj_baseline, trajectories, colors, varargin)
            s = [1,3,5,7,2,4,6]; %Subplot order
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (isempty(varargin))
                figure;
            else
                display = varargin{1};
            end
            leg={};
            ax = zeros(1,7);
            ts_baseline = timeseries(traj_baseline.qdotdot, traj_baseline.t);
            for ntraj = 1:size(trajectories,2)
                ts_trajectory = timeseries(trajectories{ntraj}.qdotdot, trajectories{ntraj}.t);
                [ts_baseline_used, ts_trajectory_used] = synchronize(ts_baseline, ts_trajectory, 'Intersection');
                st_baseline = mean(traj_baseline.t(2:end)-traj_baseline.t(1:end-1));
                st_traj = mean(trajectories{ntraj}.t(2:end)-trajectories{ntraj}.t(1:end-1));
                if (size(ts_baseline_used.Time,1)<100)
                    [ts_baseline_used, ts_trajectory_used] = synchronize(ts_baseline, ts_trajectory, 'Uniform', 'interval', (st_baseline+st_traj)*2);
                end
                ts_error = ts_trajectory_used-ts_baseline_used;
                for j = 1:IiwaRobot.n_joints
                    if (isempty(varargin))
                        ax(j) = subplot(4,2,s(j));
                    else
                        ax(j) = subplot(4,2,s(j), 'Parent', display);
                    end
                    rms_error(j) = rms(ts_error.Data(:,j)); 
                    dashedcolor = ['--', colors(ntraj)];
                    plot(ax(j), ts_error.Time', ones(length(ts_error.Time),1).*rms_error(j)', dashedcolor);
                    hold(ax(j), 'on');
                    plot(ax(j), ts_error.Time, rad2deg(ts_error.Data(:,j)), colors(ntraj), 'LineWidth', IiwaPlotter.LineWidth);
                    leg=[leg trajectories{ntraj}.name];
                    if (IiwaPlotter.plot_points)
                        plot(ax(j), ts_error.Time, rad2deg(ts_error.Data(:,j)), [colors(ntraj), '.']);
                        leg=[leg trajectories{ntraj}.name];
                    end
                    legend(ax(j), ['Error_{', trajectories{ntraj}.name, '}'], ['rms_{', trajectories{ntraj}.name, '} = ', num2str(rms_error(j),3), 'deg/s2']);
                    title(ax(j), ['J', num2str(j)])
                    ylabel(ax(j), '[deg/s2]');
                    xlabel(ax(j), 'Time [s]');
                    grid(ax(j),'on');
                end
            end
        end
        function cartesian_position_error(traj_baseline, trajectories, varargin)
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (isempty(varargin))
                figure;
            else
                display = varargin{1};
            end
            leg=cell(1, size(trajectories,2));
            ts_baseline = timeseries(traj_baseline.x, traj_baseline.t);
            
            for ntraj = 1:size(trajectories,2)
                %Calculate cartesian_error
                ts_trajectory = timeseries(trajectories{ntraj}.x, trajectories{ntraj}.t);
                [ts_baseline_used, ts_trajectory_used] = synchronize(ts_baseline, ts_trajectory, 'Intersection');
                st_baseline = mean(traj_baseline.t(2:end)-traj_baseline.t(1:end-1));
                st_traj = mean(trajectories{ntraj}.t(2:end)-trajectories{ntraj}.t(1:end-1));
                if (size(ts_baseline_used.Time,1)<100)
                    [ts_baseline_used, ts_trajectory_used] = synchronize(ts_baseline, ts_trajectory, 'Uniform', 'interval', (st_baseline+st_traj)*2);
                end
                error = zeros(size(ts_trajectory_used.Data,1), 6);
                for i=1:size(ts_trajectory_used.Data,1)
                    error(i,:) = IiwaScrewTheory.screwA2B_A(ts_baseline_used.Data(i,:), ts_trajectory_used.Data(i,:));
                end
                for i=4:6
                    ids = find(error(:,i)>pi/2);
                    error(ids,i) = error(ids,i)-pi;
                    ids = find(error(:,i)<-pi/2);
                    error(ids,i) = error(ids,i)+pi;
                    ids = find(abs(error(:,i))>pi/4);
                    error(ids,i) = 0;
                end
                error(:,1:3) = 100*error(:,1:3);
                error(:,4:6) = rad2deg(error(:,4:6));
                ts_error = timeseries(error, ts_baseline_used.Time);
                if (isempty(varargin))
                    IiwaPlotter.cartesian_error(ts_error,1, trajectories{ntraj}.name, 'cm');
                else
                    IiwaPlotter.cartesian_error(ts_error,1,trajectories{ntraj}.name, 'cm', varargin{1});
                end
                ts_error.Data = ts_error.Data(:,4:6);
                if (isempty(varargin))
                    IiwaPlotter.cartesian_error(ts_error,4, trajectories{ntraj}.name, 'deg');
                else
                    IiwaPlotter.cartesian_error(ts_error,4,trajectories{ntraj}.name, 'deg', varargin{1});
                end
            end
        end
        function cartesian_velocity_error(traj_baseline, trajectories, varargin)
            if (~iscell(trajectories))
                trajectories={trajectories};
            end
            if (isempty(varargin))
                figure;
            else
                display = varargin{1};
            end
            leg=cell(1, size(trajectories,2));
            ts_baseline = timeseries(traj_baseline.xdot, traj_baseline.t);
            for ntraj = 1:size(trajectories,2)
                %Calculate cartesian_error
                ts_trajectory = timeseries(trajectories{ntraj}.xdot, trajectories{ntraj}.t);
                [ts_baseline_used, ts_trajectory_used] = synchronize(ts_baseline, ts_trajectory, 'Intersection');
                st_baseline = mean(traj_baseline.t(2:end)-traj_baseline.t(1:end-1));
                st_traj = mean(trajectories{ntraj}.t(2:end)-trajectories{ntraj}.t(1:end-1));
                if (size(ts_baseline_used.Time,1)<100)
                    [ts_baseline_used, ts_trajectory_used] = synchronize(ts_baseline, ts_trajectory, 'Union');
                end
                error = zeros(size(ts_trajectory_used.Data,1), 6);
                for i=1:size(ts_trajectory_used.Data,1)
                    error(i,:) = IiwaScrewTheory.screwA2B_A(ts_baseline_used.Data(i,:), ts_trajectory_used.Data(i,:));
                end
                for i=4:6
                    ids = find(error(:,i)>pi/2);
                    error(ids,i) = error(ids,i)-pi;
                    ids = find(error(:,i)<-pi/2);
                    error(ids,i) = error(ids,i)+pi;
                    ids = find(abs(error(:,i))>pi/4);
                    error(ids,i) = 0;
                end
                error(:,1:3) = 100*error(:,1:3);
                error(:,4:6) = rad2deg(error(:,4:6));
                ts_error = timeseries(error, ts_baseline_used.Time);
                if (isempty(varargin))
                    IiwaPlotter.cartesian_error(ts_error,1, trajectories{ntraj}.name, 'cm/s');
                else
                    IiwaPlotter.cartesian_error(ts_error,1,trajectories{ntraj}.name, 'cm/s', varargin{1});
                end
                ts_error.Data = ts_error.Data(:,4:6);
                if (isempty(varargin))
                    IiwaPlotter.cartesian_error(ts_error,4, trajectories{ntraj}.name, 'deg/s');
                else
                    IiwaPlotter.cartesian_error(ts_error,4,trajectories{ntraj}.name, 'deg/s', varargin{1});
                end
            end
        end
        function cartesian_error(ts_error, n_start, name_traj, unit_error, varargin)
            s=[1,3,5,7,2,4,6,8];
            rms_xpos = zeros(1,3);
            ax = zeros(1,3);
            for i=n_start:n_start+2
                row = (floor((i-1)/3))+1;
                id = i - (row-1)*3;
                if (isempty(varargin))
                    ax(id) = subplot(4,2,s(i+row-1));
                else
                    display=varargin{1};
                    ax(id) = subplot(4,2,s(i+row-1), 'Parent', display);
                end
                rms_xpos(id) = rms(ts_error.Data(:,id));
                plot(ax(id), ts_error.Time, ts_error.Data(:,id), IiwaPlotter.ColorsXYZ(id), 'LineWidth', IiwaPlotter.LineWidth);
                hold(ax(id), 'on');
                dashedcolor = ['--', IiwaPlotter.ColorsXYZ(id)];
                plot(ax(id), ts_error.Time, ones(length(ts_error.Time),1)*rms_xpos(id), dashedcolor);
                grid(ax(id), 'on');
                legend(ax(id), ['Error_{', name_traj, '}'], ['rms_{', name_traj, '} = ', num2str(rms_xpos(id),3), unit_error]);
                ylabel(ax(id), ['E[', unit_error, ']']);
                xlabel(ax(id), 'Time[s]');
                title(ax(id), IiwaPlotter.CoordNames(i));
                legend(ax(id), 'Location', 'southeast');
            end
            if (isempty(varargin))
                ax(id+1) = subplot(4,2,s(i+1+row-1));
            else
                ax(id+1) = subplot(4,2,s(i+1+row-1), 'Parent', display);
            end
            hold(ax(id+1), 'on');
            xpos_euc_error = vecnorm(ts_error.Data(:,1:3)');
            rms_euc_xpos = rms(xpos_euc_error);
            plot(ax(id+1), ts_error.Time, xpos_euc_error, 'k', 'LineWidth', IiwaPlotter.LineWidth);
            dashedcolor = ['--', 'k'];
            plot(ax(id+1), ts_error.Time, ones(length(ts_error.Time),1)*rms_euc_xpos, dashedcolor);
            legend(ax(id+1), ['Error_{', name_traj, '}'], ['rms_{', name_traj, '}=', num2str(rms_euc_xpos,3), unit_error]);
            title(ax(id+1), 'Euclidean');
            ylabel(ax(id+1), ['E[', unit_error, ']']);
            xlabel(ax(id+1), 'Time[s]');
            grid(ax(id+1), 'on');
            legend(ax(id+1), 'Location', 'southeast');
        end
        function effortWithPD(traj_des, traj_comm)
            figure;
            ts_des=timeseries(traj_des.effort, traj_des.t);
            ts_comm=timeseries(traj_comm.effort, traj_comm.t);
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
                    xlabel('time [s]');
                end
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
        end
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
                        xlabel('time [s]');
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
                        xlabel('time [s]');
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
                xlabel('time [s]');
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
                    xlabel('time [s]');
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
                xlabel('time [s]');
                s = sprintf('j%d',j);
                ylabel(s);
                grid on;
            end
        end
    end
end

