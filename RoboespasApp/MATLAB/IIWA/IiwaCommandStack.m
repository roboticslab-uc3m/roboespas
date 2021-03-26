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
        function delete(obj)
            clear IiwaMsgTransformer;
        end
    end
    methods(Static)
      %Finished
        %Command
        function [traj_comm, traj_out] = MoveJ(q)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            persistent MoveJ_cli;
            persistent MoveJ_msg;
            if (isempty(MoveJ_cli) || ~isvalid(MoveJ_cli) || ~MoveJ_cli.IsServerConnected)
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
            while (~MoveJ_cli.IsServerConnected)
                pause(0.01);
            end
            resultMsg = sendGoalAndWait(MoveJ_cli, MoveJ_msg, 300);
            traj_comm = IiwaTrajectory('commanded', resultMsg.TrajectoryCommanded);
            traj_out = IiwaTrajectory('output', resultMsg.TrajectoryRead);
        end
        function [traj_comm, traj_out] = SendTraj(traj_des)
            %Interface to select a specific mode of commanding trajectories
            [traj_comm, traj_out] = IiwaCommandStack.MoveJVelTraj(traj_des);
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
            traj_out = traj_out.ChangeSampleTime(mean(traj_comm.t(2:end)-traj_comm.t(1:end-1)));
        end
        function [traj_comm, traj_out] = MoveJVelTraj(traj_des)
            %First move to first joint_position
            if (~isempty(traj_des.q))
                IiwaCommandStack.MoveJ(traj_des.q(1,:));
            end
            persistent MoveJVelTraj_cli;
            persistent MoveJVelTraj_msg;
            if (isempty(MoveJVelTraj_cli) || ~isvalid(MoveJVelTraj_cli) || strcmp(MoveJVelTraj_cli.GoalState, 'pending')==1)
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
            while (~MoveJVelTraj_cli.IsServerConnected)
                pause(0.01);
            end
            resultMsg = sendGoalAndWait(MoveJVelTraj_cli, MoveJVelTraj_msg);
            traj_comm = IiwaTrajectory('commanded', resultMsg.TrajectoryCommanded);
            traj_out = IiwaTrajectory('output', resultMsg.TrajectoryRead);
        end
        %Read state
        function [q, qdot, effort] = ReadCurrentJointState()
            sub = rossubscriber('/iiwa_command/joint_state');
            msg = receive(sub, 5);
            q = msg.Position';
            qdot = msg.Velocity';
            effort = msg.Effort';
        end
        function q = ReadCurrentJointPosition()
            [q, ~, ~] = IiwaCommandStack.ReadCurrentJointState();
        end
        function qdot = ReadCurrentJointVelocity()
            [~, qdot, ~] = IiwaCommandStack.ReadCurrentJointState();
        end
        function effort = ReadCurrentJointEffort()
            [~, ~, effort] = IiwaComandStack.ReadCurrentJointState();
        end
        %Configuration
        function SetVelocity(v)       
            %Make sure to be somewhere not near the limits of the workspace
            %before calling this function or it won't work and the
            %communication will need to be restarted
            if (v<0 || v>1)
                ME = MException('IiwaCommandStack:WrongVelocity', 'Given velocity must belong [0,1]');
                throw(ME)
            end
            rosparam("set", "/iiwa_command/velocity", v);
            pause(0.1); %Important to give time to iiwa_command node to treat the parameter change event before continue executing commands to that same ROS node
        end
        function SetAcceleration(a)
            %Make sure to be somewhere not near the limits of the workspace
            %before calling this function or it won't work and the
            %communication will need to be restarted
            if (a<0 || a>1)
                ME = MException('IiwaCommandStack:WrongAcceleration', 'Given acceleration must belong [0,1]');
                throw(ME)
            end
            rosparam("set", "/iiwa_command/acceleration", a);
            pause(0.1); %Important to give time to iiwa_command node to treat the parameter change event before continue executing commands to that same ROS node
        end
        function SetJerk(j)
            %Make sure to be somewhere not near the limits of the workspace
            %before calling this function or it won't work and the
            %communication will need to be restarted
            if (j<0 || j>1)
                ME = MException('IiwaCommandStack:WrongJerk', 'Given jerk must belong [0,1]');
                throw(ME)
            end
            rosparam("set", "/iiwa_command/jerk", j);
            pause(0.1); %Important to give time to iiwa_command node to treat the parameter change event before continue executing commands to that same ROS node
        end
        function SetControlStepSize(t)
            if (t<0 || t > 0.5)
                ME = MException('IiwaCommandStack:WrongControlStepSize', 'Given sample time must belong [0, 0.5]');
                throw(ME)
            end
            rosparam("set", "/iiwa_command/control_step_size", t);
        end
        function t = GetControlStepSize()
            t = rosparam("get", "/iiwa_command/control_step_size");
        end
        function GravityCompensationMode(mode, coordinates)
            %Función que permite interactuar con el servicio
            %'iiwa/configuration/configureSmartServo' del iiwa_stack de
            %manera sencilla. 
            %% Parámetros: 
            %* mode: Puede ser 'joint', que liberará las articulaciones del
            %robot completamente, 'cartesian', que liberará sólo ciertos
            %ejes cartesianos, o 'stop', que bloqueará el robot.
            %* coordinates: En modo 'cartesian', será un string conteniendo
            %los ejes en los que se quiere permitir el movimiento, por
            %ejemplo 'XYZABC' para permitirlo en todos los ejes, o 'X'
            %para solo permitir el desplazamiento en X. En los otros dos
            %modos, este parámetro se ignora.
            persistent config_cli;
            persistent config_msg;
            persistent controlmode_msg;
            if (isempty(config_cli) || ~isvalid(config_cli))
            	[config_cli, config_msg]=rossvcclient('iiwa/configuration/configureSmartServo');
                controlmode_msg = rosmessage('iiwa_msgs/ControlMode');
            end
            try
                switch (mode)
                    case 'joint'
                        config_msg.ControlMode=controlmode_msg.JOINTIMPEDANCE;
                        %Default values
                        damping = [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7];
                        stiffness = [10, 10, 10, 0, 0, 0, 0];
                        config_msg.JointImpedance.JointDamping.A1=damping(1);
                        config_msg.JointImpedance.JointDamping.A2=damping(2);
                        config_msg.JointImpedance.JointDamping.A3=damping(3);
                        config_msg.JointImpedance.JointDamping.A4=damping(4);
                        config_msg.JointImpedance.JointDamping.A5=damping(5);
                        config_msg.JointImpedance.JointDamping.A6=damping(6);
                        config_msg.JointImpedance.JointDamping.A7=damping(7);
                        config_msg.JointImpedance.JointStiffness.A1=stiffness(1);
                        config_msg.JointImpedance.JointStiffness.A2=stiffness(2);
                        config_msg.JointImpedance.JointStiffness.A3=stiffness(3);
                        config_msg.JointImpedance.JointStiffness.A4=stiffness(4);
                        config_msg.JointImpedance.JointStiffness.A5=stiffness(5);
                        config_msg.JointImpedance.JointStiffness.A6=stiffness(6);
                        config_msg.JointImpedance.JointStiffness.A7=stiffness(7);
                        config_cli.call(config_msg);
                    case 'cartesian'
                        config_msg.ControlMode=controlmode_msg.CARTESIANIMPEDANCE;
                        damping=[0.7 0.7 0.7 0.7 0.7 0.7 0.7];
                        stiffness=[1500 1500 1500 300 300 300 100];
                        if (isempty(coordinates))
                            ME = MException('IiwaCommandStack:WrongCoordinatesProvided', 'In cartesian gravity compensation mode one or more coordinates must be provided: X, Y, Z, A, B or C');
                            throw(ME);
                        end
                        if (contains(coordinates, 'X'))
                            stiffness(1)=1;
                        end
                        if (contains(coordinates, 'Y'))
                            stiffness(2)=1;
                        end
                        if (contains(coordinates, 'Z'))
                            stiffness(3)=1;
                        end
                        if (contains(coordinates, 'A'))
                            stiffness(4)=1;
                        end
                        if (contains(coordinates, 'B'))
                            stiffness(5)=1;
                        end
                        if (contains(coordinates, 'C'))
                            stiffness(6)=1;
                        end
                        config_msg.CartesianImpedance.CartesianDamping.X=damping(1);
                        config_msg.CartesianImpedance.CartesianDamping.Y=damping(2);
                        config_msg.CartesianImpedance.CartesianDamping.Z=damping(3);
                        config_msg.CartesianImpedance.CartesianDamping.A=damping(4);
                        config_msg.CartesianImpedance.CartesianDamping.B=damping(5);
                        config_msg.CartesianImpedance.CartesianDamping.C=damping(6);
                        config_msg.CartesianImpedance.NullspaceDamping=damping(7);
                        config_msg.CartesianImpedance.CartesianStiffness.X=stiffness(1);
                        config_msg.CartesianImpedance.CartesianStiffness.Y=stiffness(2);
                        config_msg.CartesianImpedance.CartesianStiffness.Z=stiffness(3);
                        config_msg.CartesianImpedance.CartesianStiffness.A=stiffness(4);
                        config_msg.CartesianImpedance.CartesianStiffness.B=stiffness(5);
                        config_msg.CartesianImpedance.CartesianStiffness.C=stiffness(6);
                        config_msg.CartesianImpedance.NullspaceStiffness=stiffness(7);
                        config_cli.call(config_msg);
                    case 'stop'
                            config_msg.ControlMode=controlmode_msg.POSITIONCONTROL;
                            config_cli.call(config_msg);

                    otherwise
                        ME = MException('IiwaCommandStack:WrongGravityCompensationMode', 'Gravity compensation mode must be "joint" or "cartesian"');
                        throw(ME)
                end
            catch
                ME = MException('IiwaCommandStack:ErrorGravityCompensationMode', 'Move the robot away from the workspace limits, restart the system and retry.');
                throw(ME);
            end
        end
        %Capture
        function StartCapture(~)
            persistent StartCapture_cli;
            persistent StartCapture_msg;
            if (isempty(StartCapture_cli) || ~isvalid(StartCapture_cli))
                [StartCapture_cli, StartCapture_msg] = rossvcclient('/iiwa_command/capture/start');
            end
            StartCapture_cli.call(StartCapture_msg);
        end
        function traj = StopCapture(~)
            persistent StopCapture_cli;
            persistent StopCapture_msg;
            if (isempty(StopCapture_cli) || ~isvalid(StopCapture_cli))
                [StopCapture_cli, StopCapture_msg] = rossvcclient('/iiwa_command/capture/stop');
            end
            response = StopCapture_cli.call(StopCapture_msg);
            traj = IiwaTrajectory('commanded', response.TrajectoryRead);
        end
    end
end

