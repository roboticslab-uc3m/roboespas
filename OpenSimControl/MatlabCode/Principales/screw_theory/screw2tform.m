
function H = screw2tform(twist, theta)
    %#codegen
    %Screw2Tform
    v = twist(1:3); 
    w = twist(4:6);
    if norm(w) == 0 % only translation
       r = eye(3);
       p = v*theta;
    else
        %axang2rotm
        r = axang2rotm([w' theta]);
        %cross
        p = (eye(3)-r)*(cross(w,v)); % for only rotation joint.     
    end
    H = [r, p; [0 0 0 1]];
end

