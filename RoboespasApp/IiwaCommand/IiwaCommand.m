classdef IiwaCommand < handle
    properties
        PlotFeedback = false;
    end
    
    methods (Static, Access='public')
        function [q, effort] = ReadCurrentJointState()
            sub = rossubscriber('/iiwa_command/joint_state');
            msg = receive(sub, 5);
            q = msg.Position';
            effort = msg.Effort';
        end
        function q = ReadCurrentJointPosition()
            [q, ~] = IiwaCommand.ReadCurrentJointState();
        end
        function effort = ReadCurrentJointEffort()
            [~, effort] = IiwaComand.ReadCurrentJointState();
        end
        function SetVelocity(v)
            if (v<0 || v>1)
                ME = MException('IiwaCommand:WrongVelocity', 'Given velocity must belong [0,1]');
                throw(ME)
            end
            rosparam("set", "/iiwa_command/velocity", v);
        end
        function SetSampleTime(t) %Also called control_step_time
            if (t<0 || t>0.100)
                ME = MException('IiwaCommand:WrongSampleTime', 'Given sample time must belong [0, 0.1]');
                throw(ME)
            end
            rosparam("set", "/iiwa_command/control_step_size", t);
        end
        function [traj_comm, traj_output] = MoveJ(q)
            persistent MoveJASrv_cli MoveJASrv_msg initialized;
            if (~initialized)
                [MoveJASrv_cli, MoveJASrv_msg] = rosactionclient('MoveJ');
                MoveJASrv_cli.ActivationFcn = @(~) disp('MoveJ action server active');
                if (IiwaCommand.PlotFeedback)
                    figure; hold on;
                    MoveJASrv_cli.FeedbackFcn = @(~,msg) IiwaPlotter.joint_position(msg.JointState.Position', msg.TimeFromStart);
                else
                    MoveJASrv_cli.FeedbackFcn = @(~,msg) (1);
                end
                MoveJASrv_cli.ResultFcn = @(~,msg) disp('MoveJ action server result received');
                initialized=true;
            end
            MoveJASrv_msg.JointPosition=q;
            resultMsg = sendGoalAndWait(MoveJASrv_cli, MoveJASrv_msg);
            traj_comm = IiwaTrajectory('commanded', resultMsg.TrajectoryCommanded);
            traj_output = IiwaTrajectory('output', resultMsg.TrajectoryRead);
        end
    end
end

