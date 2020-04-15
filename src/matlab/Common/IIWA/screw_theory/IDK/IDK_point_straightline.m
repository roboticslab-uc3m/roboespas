function q_end = IDK_point_straightline(q_ini, displacement)
    [xpos_ini, xori_ini]=forward_kinematics(q_ini);
    npoints=500;
    t=linspace(0, 10, npoints);
    xpos=[linspace(xpos_ini(1), xpos_ini(1)+displacement(1), npoints); ...
                     linspace(xpos_ini(2), xpos_ini(2)+displacement(2), npoints); ...
                     linspace(xpos_ini(3), xpos_ini(3)+displacement(3), npoints)];
    % !! Does NOT work for displacements in orientation, use
    % IDK_point_straightline2 and absolute pos/ori, instead of relative
    % ones
    xori=[linspace(xori_ini(1), xori_ini(1)+displacement(4), npoints); ...
                     linspace(xori_ini(2), xori_ini(2)+displacement(5), npoints); ...
                     linspace(xori_ini(3), xori_ini(3)+displacement(6), npoints)];
    data_output=IDK_trajectory(xpos, xori, t, q_ini);    
    q_end = data_output.q(:,end);
end

