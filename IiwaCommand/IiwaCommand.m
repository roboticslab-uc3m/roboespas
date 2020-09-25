classdef IiwaCommand < handle
    %IIWACOMMAND Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        plot_feedback=0;
        smoothing_minimum = 1e-8;
        qdotdot_max = 1;
    end
    
    methods (Static)
        function obj = IiwaCommand()
            init_ros;
        end
        function SetVelocityFactor(v)
            if v<0 || v>1
                disp('Error, v must be between 0 and 1')
            else
            	rosparam('set', '/iiwa_command/velocity', v);
            end
        end
        function qdot_max = GetCurrentMaximumVelocity()
            qdot_factor = rosparam('get', '/iiwa_command/velocity');
            qdot_maximum = deg2rad(cell2mat(rosparam('get', '/iiwa/limits/joint_velocity')))';
            qdot_max = qdot_factor*qdot_maximum;
        end
        function control_step_size = GetControlStepSize()
            control_step_size= rosparam('get', '/iiwa_command/control_step_size');
        end
        function q = GetCurrentJointPosition()
            persistent initialized;
            persistent q_sub;
            if (isempty(initialized))
                q_sub = rossubscriber('/iiwa_command/joint_state');
                initialized = 1;
                
            end
            q = q_sub.receive.Position;
        end
        function [traj_comm, traj_output] = MoveJ(q)  
            [MoveJASrv_cli, MoveJASrv_msg] = rosactionclient('MoveJ');
            MoveJASrv_cli.ActivationFcn = @(~) disp('MoveJ action server active');
            MoveJASrv_cli.ResultFcn = @(~,msg) disp('MoveJ action server result received');
            if (IiwaCommand.plot_feedback)
                figure; hold on;
                MoveJASrv_cli.FeedbackFcn = @(~,msg) IiwaPlotter.joint_position(msg.JointState.Position', msg.TimeFromStart);
            else
                MoveJASrv_cli.FeedbackFcn = @(~,msg) (1);
            end
            MoveJASrv_msg.JointPosition=q;
            resultMsg = sendGoalAndWait(MoveJASrv_cli, MoveJASrv_msg);
            traj_comm = IiwaTrajectory('commanded', resultMsg.TrajectoryCommanded);
            traj_output = IiwaTrajectory('output', resultMsg.TrajectoryRead);
        end
        function [traj_comm, traj_output] = MoveJ2(q)
            if (size(q) == [1 IiwaRobot.n_joints])
                q=q';
            end
            if (size(q) ~= [IiwaRobot.n_joints 1])
                disp('Wrong size q');
            end
            %Read parameters
            q_curr = IiwaCommand.GetCurrentJointPosition();
            qdot_max = IiwaCommand.GetCurrentMaximumVelocity();
            control_step_size = IiwaCommand.GetControlStepSize();
            %Calc increment
            q_inc = q - q_curr;
            % Find t_needed to reach position
            t_needed = q_inc./qdot_max;
            t_max = max(abs(t_needed));
            t_total = ceil(t_max/control_step_size)*control_step_size;
            % Build trajectory
            n_points = round(t_total/control_step_size) + 1;   
            traj_straight = IiwaTrajectory('movej_straight', n_points);
            x = linspace(0,1,n_points);
            traj_straight.q = (q_curr + x.*q_inc)';
            traj_straight.t = (0:control_step_size:t_total)';
            traj_spline = IiwaTrajectoryGeneration.BoundedSplineTrajectory(traj_straight, IiwaCommand.smoothing_minimum);
            [traj_comm, traj_output] = IiwaCommand.MoveJTrajectory(traj_spline);
        end
        function [traj_comm, traj_output] = MoveJTrajectory(traj_sent)
            [MoveJTrajectoryASrv_cli, MoveJTrajectoryASrv_msg] = rosactionclient('IiwaCommandFriNode'); %rosactionclient('MoveJTrajectory');
            MoveJTrajectoryASrv_cli.ActivationFcn = @(~) disp('MoveJTrajectory action server active');
            MoveJTrajectoryASrv_cli.ResultFcn = @(~,res) disp('MoveJTrajectory result received');
            if (IiwaCommand.plot_feedback)
                figure; hold on;
                MoveJTrajectoryASrv_cli.FeedbackFcn = @(~,msg) IiwaPlotter.joint_position(msg.JointState.Position', msg.TimeFromStart);
            else
                MoveJTrajectoryASrv_cli.FeedbackFcn = @(~,msg) (1);
            end
            MoveJTrajectoryASrv_msg.TrajectoryDesired = IiwaMsgTransformer.toJointTraj(traj_sent);
            resultMsg = sendGoalAndWait(MoveJTrajectoryASrv_cli, MoveJTrajectoryASrv_msg);
            traj_comm = IiwaTrajectory('commanded', resultMsg.TrajectoryCommanded);
            traj_comm = IiwaTrajectoryGeneration.FillVelocityAndAcceleration(traj_comm);
            traj_output = IiwaTrajectory('output', resultMsg.TrajectoryRead);
            traj_output = IiwaTrajectoryGeneration.FillVelocityAndAcceleration(traj_output);
        end
    end
end

