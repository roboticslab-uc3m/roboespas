function angvel = substract_ori(xori_A, xori_B, time)
% Function that transform two set of euler angles into rotation matrices,
% calculate the needed rotation from the first to the second, and then
% express that rotation again in euler angles, transforming it into the
% space frame. Finally, it divides by a given time, as this function is
% thought to be used to calculate velocities.
    %Rotation matrices from space frame to both frames: current and next
    R_SA=eul2rotm(xori_A, 'XYZ');
    R_SB=eul2rotm(xori_B, 'XYZ');
    %Rotation matrix from A to B
    R_AB=R_SA'*R_SB;
    %Express this rotation in axis-angle form
    axang4_AB_A=rotm2axang(R_AB); %[axis_x, axis_y, axis_z, angle]
    axang3_AB_A=axang4_AB_A(1:3)*axang4_AB_A(4); %[axis_x*angle, axis_y*angle, axis_z*angle]
    %Express this rotation in the space frame, as the previous was
    %expressed in the A frame, just multiply by R_SA
    axang3_AB_S=R_SA*axang3_AB_A';
    %Divide by the time
    angvel=axang3_AB_S'/time;
end

