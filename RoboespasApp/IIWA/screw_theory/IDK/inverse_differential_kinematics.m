function [qdot, q_next] = inverse_differential_kinematics(q_curr, x_next, time)
    format long;
    %Get IIWA data:
    [~, ~, ~, Twists, ~, qmax, qdotmax] = data_IIWA;
    %IDK
    % Calculate current cartesian position using forward kinematics
    [x_curr(1:3), x_curr(4:6)]=forward_kinematics(q_curr');
    % Calculate space jacobian
    Jst=GeoJacobianS(Twists, q_curr);
    % Calculate cartesian velocity - Position
    v_xyz=(x_next(1:3)-x_curr(1:3))/time;
    % Calculate cartesian velocity - Orientation
    w_xyz=substract_ori(x_curr(4:6), x_next(4:6), time);
    % Transform velocity in screw form
    v_screw=v_xyz-cross(w_xyz,x_curr(1:3));
    w_screw=w_xyz;
    cart_vel_screw=[v_screw, w_screw]';
    % Calculate joint velocity with Moore-Penrose pseudojacobian
    Jst_i=pinv(Jst);%Jst'*inv(Jst*Jst');%pinv(Jst);
    qdot=(Jst_i*cart_vel_screw)';
    % Limit velocity to joint_vel_limits
    qdot=limit_qdot(qdot, qdotmax);
    % Calculate joint_position that the robot should achieve moving at that
    % velocity during the given time
    q_next=q_curr(1:size(qdot,2))+qdot*time;
    % Limit position to limits
    q_next=limit_q(q_next, qmax, -qmax);
end

