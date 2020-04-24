classdef IiwaMsgTransformer < handle
    methods(Static)
        function iiwa_traj = toIiwaTrajectory(name, input)
            persistent initialized
            persistent fromJointTrajSrv_cli fromJointTrajSrv_msg;
            persistent fromJointStateVecSrv_cli fromJointStateVecSrv_msg;
            iiwa_traj=IiwaTrajectory();
            if (isempty(initialized))
                [fromJointTrajSrv_cli, fromJointTrajSrv_msg] = rossvcclient('/msg_transform_helper/from_joint_traj');
                [fromJointStateVecSrv_cli, fromJointStateVecSrv_msg] = rossvcclient('/msg_transform_helper/from_joint_state_vec');
                initialized='true';
            end
            if isa(input,'robotics.ros.custom.msggen.iiwa_command.FromJointTrajectoryResponse') || ...
                isa(input, 'robotics.ros.custom.msggen.iiwa_command.FromJointStateVecResponse')
                iiwa_traj.t=input.Stamps;
                iiwa_traj.q=reshape(input.Positions, 7, size(input.Positions,1)/7, 1)';
                iiwa_traj.qdot=reshape(input.Velocities, 7, size(input.Velocities,1)/7, 1)';
                iiwa_traj.qdotdot=reshape(input.Accelerations, 7, size(input.Accelerations,1)/7, 1)';
                iiwa_traj.effort=reshape(input.Efforts, 7, size(input.Efforts,1)/7, 1)';
            elseif isa(input, 'robotics.ros.msggen.trajectory_msgs.JointTrajectory')
                fromJointTrajSrv_msg.JointTrajectory=input;
                fromJointTrajSrv_res=fromJointTrajSrv_cli.call(fromJointTrajSrv_msg);
                iiwa_traj=IiwaMsgTransformer.toIiwaTrajectory(name, fromJointTrajSrv_res);
            elseif isa(input, 'robotics.ros.msggen.sensor_msgs.JointState')
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
    end
end

