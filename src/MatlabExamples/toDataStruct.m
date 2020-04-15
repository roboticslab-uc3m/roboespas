function data = toDataStruct(input)
    if isa(input,'robotics.ros.custom.msggen.iiwa_command.FromJointTrajectoryResponse') || ...
            isa(input, 'robotics.ros.custom.msggen.iiwa_command.FromJointStateVecResponse')
        data.t=input.Stamps';
        data.q=reshape(input.Positions, 7, size(input.Positions,1)/7, 1);
        data.qdot=reshape(input.Velocities, 7, size(input.Velocities,1)/7, 1);
        data.qdotdot=reshape(input.Accelerations, 7, size(input.Accelerations,1)/7, 1);
        data.effort=reshape(input.Efforts, 7, size(input.Efforts,1)/7, 1);
    end
end

