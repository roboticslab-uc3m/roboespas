classdef IiwaCommandFRI < handle
    %IIWACOMMANDFRI Summary of this class goes here
    %   Detailed explanation goes here
    %Position to change velocity: [-15, 30, 30, -60, 0, 90, 30]
    properties (Constant)
        PlotFeedback=0;
        Acceleration = 90; %radians
    end
    
    methods
        function obj = IiwaCommandFRI()
            %IIWACOMMANDFRI Construct an instance of this class
            %   Detailed explanation goes here
        end
        function delete(obj)
            clear IiwaMsgTransformer;
        end
    end
    methods(Static)
      %Finished
        %Command
        function [traj_comm, traj_out, traj_theory] = MoveJ(q)
            qini = IiwaCommandFRI.ReadCurrentJointPosition();
            qend = q;
            css = IiwaCommandFRI.GetControlStepSize();
            qdot_max = IiwaCommandFRI.GetMaximumJointVelocity();
            v = IiwaCommandFRI.GetVelocity();
            qdot = qdot_max.*v;
            qdotdot = IiwaCommandFRI.Acceleration*ones(1,7);
            traj_theory = IiwaTrajectoryGeneration.TrapezoidalVelocityProfileTrajectory(qini, qend, css, qdot, qdotdot, 'movej');
            [traj_comm, traj_out] = IiwaCommandFRI.MoveJTraj(traj_theory);
        end
        function [traj_comm, traj_out] = SendTraj(traj_des)
            %Interface to select a specific mode of commanding trajectories
            [traj_comm, traj_out] = IiwaCommandFRI.MoveJTraj(traj_des);
        end
        function [traj_comm, traj_out] = MoveJTraj(traj_des)
            persistent MoveJTraj_cli;
            persistent MoveJTraj_msg;
            if (isempty(MoveJTraj_cli) || ~isvalid(MoveJTraj_cli))
                [MoveJTraj_cli, MoveJTraj_msg] = rosactionclient('MoveJTraj');
                MoveJTraj_cli.ActivationFcn = @(~) disp('MoveJTrajectory action server active');
                MoveJTraj_cli.ResultFcn = @(~,res) disp('MoveJTrajectory result received');
            end
            if (IiwaCommandFRI.PlotFeedback)
                figure; hold on;
                MoveJTraj_cli.FeedbackFcn = @(~,msg) IiwaPlotter.joint_position(msg.JointState.Position', msg.TimeFromStart);
            else
                MoveJTraj_cli.FeedbackFcn = @(~,msg) (1);
            end
            MoveJTraj_msg.TrajectoryDesired = IiwaMsgTransformer.toJointTraj(traj_des);
            while (~MoveJTraj_cli.IsServerConnected)
                pause(0.01);
            end
            resultMsg = sendGoalAndWait(MoveJTraj_cli, MoveJTraj_msg);
            traj_comm = IiwaTrajectory('commanded', resultMsg.TrajectoryCommanded);
            traj_out = IiwaTrajectory('output', resultMsg.TrajectoryRead);
        end
        
        function [traj_comm, traj_out] = MoveJCartVelTraj(traj_des)
            %First move to first joint_position
%             for i=1:traj_des.npoints
%                 traj_des.xdot(i,:) = IiwaScrewTheory.tfscrew_A2S(traj_des.xdot(i,:), traj_des.x(i,:));
%             end
            if (~isempty(traj_des.q))
                IiwaCommandFRI.MoveJ(traj_des.q(1,:));
            end
            persistent MoveJCartVelTraj_cli;
            persistent MoveJCartVelTraj_msg;
            if (isempty(MoveJCartVelTraj_cli) || ~isvalid(MoveJCartVelTraj_cli) || strcmp(MoveJCartVelTraj_cli.GoalState, 'pending')==1)
                [MoveJCartVelTraj_cli, MoveJCartVelTraj_msg] = rosactionclient('MoveJTrajCartVel');
                MoveJCartVelTraj_cli.ActivationFcn = @(~) disp('MoveJCartVelTrajectory action server active');
                MoveJCartVelTraj_cli.ResultFcn = @(~,res) disp('MoveJCartVelTrajectory result received');
            end
            if (IiwaCommandStack.PlotFeedback)
                figure; hold on;
                MoveJCartVelTraj_cli.FeedbackFcn = @(~,msg) IiwaPlotter.joint_position(msg.JointState.Position', msg.TimeFromStart);
            else
                MoveJCartVelTraj_cli.FeedbackFcn = @(~,msg) (1);
            end
            MoveJCartVelTraj_msg.CartVelDesired = IiwaMsgTransformer.toTwistStampedVec(traj_des.xdot', traj_des.t);
            MoveJCartVelTraj_msg.CartPosDesired = IiwaMsgTransformer.toTwistStampedVec(traj_des.x', traj_des.t);
            while (~MoveJCartVelTraj_cli.IsServerConnected)
                pause(0.01);
            end
            traj_des.name = 'desired';
            resultMsg = sendGoalAndWait(MoveJCartVelTraj_cli, MoveJCartVelTraj_msg);
            traj_comm = IiwaTrajectory('commanded', resultMsg.TrajectoryCommanded);
            traj_out = IiwaTrajectory('output', resultMsg.TrajectoryRead);
        end
        %Read state
        function [q, effort] = ReadCurrentJointState()
            sub = rossubscriber('/iiwa_command/joint_state');
            msg = receive(sub, 5);
            q = msg.Position';
            effort = msg.Effort';
        end
        function q = ReadCurrentJointPosition()
            [q, ~] = IiwaCommandFRI.ReadCurrentJointState();
        end
        function effort = ReadCurrentJointEffort()
            [~, effort] = IiwaComandFRI.ReadCurrentJointState();
        end
        %Configuration
        function t = GetControlStepSize()
            t = rosparam("get", "/iiwa_command/control_step_size");
        end
        function SetControlStepSize(css)
            ME = MException('IiwaCommandFRI:SetControlStepSize', 'ControlStepSize cannot be changed in FRI mode');
            throw(ME);
        end
        function qdot_max = GetMaximumJointVelocity()
            qdot_max = deg2rad(cell2mat(rosparam("get", "/iiwa/limits/joint_velocity")));
        end
        function qinc_max = GetMaximumJointPositionIncrement()
            qdot_max = IiwaCommandFRI.GetMaximumJointVelocity();
            css = IiwaCommandFRI.GetControlStepSize();
            qinc_max = qdot_max*css;
        end
        function SetKp(kp)
            if (kp<0 || kp>1)
                ME = MException('IiwaCommandStack:InvalidKp', 'Given kp must belong [0, 1]');
                throw(ME)
            end
            rosparam('set', '/iiwa_command/control_cart_vel/kp', kp);
        end
        function SetVelocity(v)
            %disp('Velocity not available in FRI mode');
        end
        function SetAcceleration(a)
            %disp('Acceleration not available in FRI mode');
        end
        function SetJerk(j)
            %disp('Acceleration not available in FRI mode');
        end
        %Capture
        function StartCapture(~)
%             persistent StartCapture_cli;
%             persistent StartCapture_msg;
%             if (isempty(StartCapture_cli) || ~isvalid(StartCapture_cli))
%                 [StartCapture_cli, StartCapture_msg] = rossvcclient('/iiwa_command/capture/start');
%             end
%             StartCapture_cli.call(StartCapture_msg);
            disp('TODO:StartCapture');
        end
        function traj = StopCapture(~)
%             persistent StopCapture_cli;
%             persistent StopCapture_msg;
%             if (isempty(StopCapture_cli) || ~isvalid(StopCapture_cli))
%                 [StopCapture_cli, StopCapture_msg] = rossvcclient('/iiwa_command/capture/stop');
%             end
%             response = StopCapture_cli.call(StopCapture_msg);
%             traj = IiwaTrajectory('commanded', response.TrajectoryRead);
            traj = IiwaTrajectory();
            disp('TODO:StopCapture');
        end
        function GravityCompensationMode(~, ~)
            ME = MException('IiwaCommandFRI:GravityCompensationMode', 'GravityCompensationMode not available in FRI mode');
            throw(ME);
        end
            
    end
end

