function [cartesian_forces] = FD(joint_torques, joint_torques_vacio, joint_positions)
    n=min(size(joint_torques,2), size(joint_torques_vacio,2));
    n=min(n,size(joint_positions,2));
    [~,~,~, Twist, ~,~] = data_IIWA();
    for i=1:n
        diff_torques=joint_torques(:,i)-joint_torques_vacio(:,i);
        current_joint_pos=joint_positions(:,i);
        TwMag=[Twist; current_joint_pos'];
        J=GeoJacobianS(TwMag);
        cartesian_forces(:,i)=pinv(J')*diff_torques;
    end
end

