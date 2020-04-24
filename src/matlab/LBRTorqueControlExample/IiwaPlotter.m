classdef IiwaPlotter < handle
    properties
        Property1
    end
    
    methods
        function obj = IiwaPlotter()
        end
    end
    methods(Static)
        function effortWithPD(traj_des, traj_comm)
            figure;
            pdTorque=traj_comm.effort - traj_des.effort;
            feedForwardTorque = traj_des.effort;
            for j = 1:7
                subplot(7,1,j);
                plot(traj_des.t, feedForwardTorque(:,j), 'm'); hold on
                plot(traj_des.t, pdTorque(:,j), 'r');
                plot(traj_des.t, pdTorque(:,j) + feedForwardTorque(:,j), 'k');
                if j == 1
                    legend('feed-forward torque (N)','PD torque','total torque');
                end
                if j == 7
                    xlabel('time (s)');
                end
                s = sprintf('jnt\\_%d',j);
                ylabel(s);
                grid on;
            end
        end
        
        function effortPoint(as_feedback_msg)
           % Time between points plotted. You may need to change this depending on 
           % your computer's resources
           time_plot=0.05; %seconds
           if (mod(as_feedback_msg.TimeFromStart, time_plot)==0)
                for i=1:7
                    subplot(7,1,i);
                    plot(as_feedback_msg.TimeFromStart, as_feedback_msg.PointCommanded.Effort(i), '.b');
                    hold on;
                    plot(as_feedback_msg.TimeFromStart, as_feedback_msg.JointState.Effort(i), '.r');
                end
           end
        end
    end
end

