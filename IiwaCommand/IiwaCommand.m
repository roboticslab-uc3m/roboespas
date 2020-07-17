classdef IiwaCommand
    properties
    end
    
    methods(Static)
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
    end
end

