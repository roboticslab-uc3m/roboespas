classdef IiwaCommandStack < handle
    %IIWACOMMANDSTACK Summary of this class goes here
    %   Detailed explanation goes here
    %Position to change velocity: [-15, 30, 30, -60, 0, 90, 30]
    properties (Constant)
        PlotFeedback=0;
    end
    
    methods
        function obj = IiwaCommandStack()
            %IIWACOMMANDSTACK Construct an instance of this class
            %   Detailed explanation goes here
        end
    end
    methods(Static)
        function result = ChangeVelAccJerk(v, a, j)
            if (v<=0 || a<=0 || j<=0)
                ME = MException('IiwaCommandStack:ChangeVelAccJerk', 'Inputs must be >=0');
                throw(ME);
            end
            if (v>1 || a>1 || j>1)
                ME = MException('IiwaCommandStack:ChangeVelAccJerk', 'Inputs must be <1');
                throw(ME);
            end
            persistent PathPar_cli;
            persistent PathPar_msg;
            if (isempty(PathPar_cli) || ~isvalid(PathPar_cli))
                [PathPar_cli, PathPar_msg] = rossvcclient('/iiwa/configuration/pathParameters');
            end
            PathPar_msg.JointRelativeVelocity=v;
            PathPar_msg.JointRelativeAcceleration=a;
            PathPar_msg.OverrideJointAcceleration=j;
            result = call(PathPar_cli, PathPar_msg);
        end
        function [traj_comm, traj_out] = MoveJ(q)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            persistent MoveJ_cli;
            persistent MoveJ_msg;
            if (isempty(MoveJ_cli) || ~isvalid(MoveJ_cli))
                [MoveJ_cli, MoveJ_msg] = rosactionclient('MoveJ');
                MoveJ_cli.ActivationFcn = @(~) disp('MoveJ action server active');
                MoveJ_cli.ResultFcn = @(~,msg) disp('MoveJ action server result received');
            end
            if (IiwaCommandStack.PlotFeedback)
                figure; hold on;
                MoveJ_cli.FeedbackFcn = @(~,msg) IiwaPlotter.joint_position(msg.JointState.Position', msg.TimeFromStart);
            else
                MoveJ_cli.FeedbackFcn = @(~,msg) (1);
            end
            MoveJ_msg.JointPosition=q;
            resultMsg = sendGoalAndWait(MoveJ_cli, MoveJ_msg);
            traj_comm = IiwaTrajectory('commanded', resultMsg.TrajectoryCommanded);
            traj_out = IiwaTrajectory('output', resultMsg.TrajectoryRead);
        end
        function [traj_comm, traj_out] = MoveJTraj(traj_des)
            %First move to first joint_position
            if (~isempty(traj_des.q))
                IiwaCommandStack.MoveJ(traj_des.q(1,:));
            end
            persistent MoveJTraj_cli;
            persistent MoveJTraj_msg;
            if (isempty(MoveJTraj_cli) || ~isvalid(MoveJTraj_cli))
                [MoveJTraj_cli, MoveJTraj_msg] = rosactionclient('MoveJTraj');
                MoveJTraj_cli.ActivationFcn = @(~) disp('MoveJTrajectory action server active');
                MoveJTraj_cli.ResultFcn = @(~,res) disp('MoveJTrajectory result received');
            end
            if (IiwaCommandStack.PlotFeedback)
                figure; hold on;
                MoveJTraj_cli.FeedbackFcn = @(~,msg) IiwaPlotter.joint_position(msg.JointState.Position', msg.TimeFromStart);
            else
                MoveJTraj_cli.FeedbackFcn = @(~,msg) (1);
            end
            MoveJTraj_msg.TrajectoryDesired = IiwaMsgTransformer.toJointTraj(traj_des);
            resultMsg = sendGoalAndWait(MoveJTraj_cli, MoveJTraj_msg);
            traj_comm = IiwaTrajectory('commanded', resultMsg.TrajectoryCommanded);
            traj_out = IiwaTrajectory('output', resultMsg.TrajectoryRead);
        end
        function [traj_comm, traj_out] = MoveJVelTraj(traj_des)
            %First move to first joint_position
            if (~isempty(traj_des.q))
                IiwaCommandStack.MoveJ(traj_des.q(1,:));
            end
            persistent MoveJVelTraj_cli;
            persistent MoveJVelTraj_msg;
            if (isempty(MoveJVelTraj_cli) || ~isvalid(MoveJVelTraj_cli))
                [MoveJVelTraj_cli, MoveJVelTraj_msg] = rosactionclient('MoveJVelTraj');
                MoveJVelTraj_cli.ActivationFcn = @(~) disp('MoveJVelTrajectory action server active');
                MoveJVelTraj_cli.ResultFcn = @(~,res) disp('MoveJVelTrajectory result received');
            end
            if (IiwaCommandStack.PlotFeedback)
                figure; hold on;
                MoveJVelTraj_cli.FeedbackFcn = @(~,msg) IiwaPlotter.joint_position(msg.JointState.Position', msg.TimeFromStart);
            else
                MoveJVelTraj_cli.FeedbackFcn = @(~,msg) (1);
            end
            MoveJVelTraj_msg.TrajectoryDesired = IiwaMsgTransformer.toJointTraj(traj_des);
            resultMsg = sendGoalAndWait(MoveJVelTraj_cli, MoveJVelTraj_msg);
            traj_comm = IiwaTrajectory('commanded', resultMsg.TrajectoryCommanded);
            traj_out = IiwaTrajectory('output', resultMsg.TrajectoryRead);
        end
    end
end

