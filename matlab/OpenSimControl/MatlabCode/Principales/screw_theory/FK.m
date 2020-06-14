function pose_vector= FK(joint_trajectory)
% Function that calculates the forward kinematics for a whole trajectory
% Orientations in quaternions in the form w, a, b, c (not a, b, c, w as
% provided by iiwa solver)
    [~, ~, ~, Twist, Hst0, ~] = data_IIWA;
    pose_vector=zeros(6, size(joint_trajectory,2));
    for i=1:size(joint_trajectory,2)
        % Build TwMag for each point
        joint_pos=joint_trajectory(:,i);
        TwMag=[Twist; joint_pos(1:size(Twist,2))'];
        % Calculate HstR (4by4 matrix) for that joint position
        HstR = ForwardKinematicsPOE(TwMag);
        % Transform the matrix into [pos; eul] form (1by6 vec)
        Hst=HstR*Hst0;
        pose_vector(1:3,i)=Hst(1:3,4);
        %orientations(:,i)=rotm2quat(Hst(1:3,1:3));
        pose_vector(4:6,i)=rotm2eul(Hst(1:3, 1:3),'XYZ');
    end
end

