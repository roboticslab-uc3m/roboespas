classdef IiwaMsgTransformer < handle
    methods(Static)
        function iiwa_traj = toIiwaTrajectory(name, input, varargin)
            persistent initialized
            persistent fromJointTrajSrv_cli fromJointTrajSrv_msg;
            persistent fromJointStateVecSrv_cli fromJointStateVecSrv_msg;
            persistent fromTwistStampedVecSrv_cli fromTwistStampedVecSrv_msg;
            iiwa_traj=IiwaTrajectory();
            if (isempty(initialized))
                [fromJointTrajSrv_cli, fromJointTrajSrv_msg] = rossvcclient('/msg_transform_helper/from_joint_traj');
                [fromJointStateVecSrv_cli, fromJointStateVecSrv_msg] = rossvcclient('/msg_transform_helper/from_joint_state_vec');                
                [fromTwistStampedVecSrv_cli, fromTwistStampedVecSrv_msg] = rossvcclient('/msg_transform_helper/from_twist_stamped_vec');
                initialized='true';
            end
            if (length(varargin)==1)
                fromTwistStampedVecSrv_msg.TwistStampedVec = input;
                x_res = fromTwistStampedVecSrv_cli.call(fromTwistStampedVecSrv_msg);
                iiwa_traj.t=x_res.Stamps;
                iiwa_traj.x=reshape(x_res.Values, 6, size(x_res.Values,1)/6, 1)';
                fromTwistStampedVecSrv_msg.TwistStampedVec = varargin{1};
                xdot_res = fromTwistStampedVecSrv_cli.call(fromTwistStampedVecSrv_msg);
                iiwa_traj.xdot=reshape(xdot_res.Values, 6, size(xdot_res.Values,1)/6, 1)';
                iiwa_traj.npoints=size(iiwa_traj.x,1);
            elseif isa(input,'robotics.ros.custom.msggen.iiwa_command.FromJointTrajectoryResponse') || ...
                isa(input, 'robotics.ros.custom.msggen.iiwa_command.FromJointStateVecResponse')
                iiwa_traj.t=input.Stamps;
                iiwa_traj.q=reshape(input.Positions, 7, size(input.Positions,1)/7, 1)';
                iiwa_traj.qdot=reshape(input.Velocities, 7, size(input.Velocities,1)/7, 1)';
                iiwa_traj.qdotdot=reshape(input.Accelerations, 7, size(input.Accelerations,1)/7, 1)';
                iiwa_traj.effort=reshape(input.Efforts, 7, size(input.Efforts,1)/7, 1)';
                iiwa_traj.npoints=size(iiwa_traj.q,1);
            elseif isa(input, 'robotics.ros.msggen.trajectory_msgs.JointTrajectory')
                %De jointTrajectory a iiwaTrajectory
                fromJointTrajSrv_msg.JointTrajectory=input;
                fromJointTrajSrv_res=fromJointTrajSrv_cli.call(fromJointTrajSrv_msg);
                iiwa_traj=IiwaMsgTransformer.toIiwaTrajectory(name, fromJointTrajSrv_res);
                iiwa_traj.npoints=size(iiwa_traj.q,1);
            elseif isa(input, 'robotics.ros.msggen.sensor_msgs.JointState')
                %De vector de joint states a iiwatrajectory
                fromJointStateVecSrv_msg.JointStateVec=input;
                fromJointStateSrv_res=fromJointStateVecSrv_cli.call(fromJointStateVecSrv_msg);
                iiwa_traj=IiwaMsgTransformer.toIiwaTrajectory(name, fromJointStateSrv_res);
            end
            iiwa_traj.name=name;
        end
        function joint_traj = toJointTraj(input)
            persistent initialized;
            persistent toJointTrajSrv_cli toJointTrajSrv_msg;
            if (isempty(initialized))
                [toJointTrajSrv_cli, toJointTrajSrv_msg] = rossvcclient('/msg_transform_helper/to_joint_traj');
                initialized='true';
            end
            toJointTrajSrv_msg.JointNames={'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'};
            toJointTrajSrv_msg.Efforts=reshape(input.effort', size(input.effort,1)*size(input.effort,2), 1);
            toJointTrajSrv_msg.Positions=reshape(input.q', size(input.q,1)*size(input.q,2), 1);
            toJointTrajSrv_msg.Velocities=reshape(input.qdot', size(input.qdot,1)*size(input.qdot,2), 1);
            toJointTrajSrv_msg.Accelerations=reshape(input.qdotdot', size(input.qdotdot,1)*size(input.qdotdot,2), 1);
            toJointTrajSrv_msg.Stamps=input.t;
            jointTrajMsg=toJointTrajSrv_cli.call(toJointTrajSrv_msg);
            joint_traj=jointTrajMsg.JointTrajectory;
        end
        function joint_state = toJointState(input)
            persistent initialized;
            persistent toJointStateVecSrv_cli toJointStateVecSrv_msg;
            if (isempty(initialized))
                [toJointStateVecSrv_cli, toJointStateVecSrv_msg] = rossvcclient('/msg_transform_helper/to_joint_state_vec');
                initialized='true';
            end
            joint_state='not implemented';
            % TODO: Implement
        end
        function twist_stamped_vec = toTwistStampedVec(values, stamps)
            persistent initialized;
            persistent toTwistStampedVecSrv_cli toTwistStampedVecSrv_msg;
            if (isempty(initialized))
                [toTwistStampedVecSrv_cli, toTwistStampedVecSrv_msg] = rossvcclient('/msg_transform_helper/to_twist_stamped_vec');
                initialized='true';
            end
            toTwistStampedVecSrv_msg.Stamps = stamps;
            toTwistStampedVecSrv_msg.Values = reshape(values, size(values,1)* size(values,2),1);
            twistStampedVecMsg = toTwistStampedVecSrv_cli.call(toTwistStampedVecSrv_msg);
            twist_stamped_vec = twistStampedVecMsg.TwistStampedVec;
        end
    end
end

