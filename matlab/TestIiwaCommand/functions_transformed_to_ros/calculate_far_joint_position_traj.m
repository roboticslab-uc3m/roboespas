clear all
q_ini=[0 0 0 0 0 0 0];
q_goal=[1.2 -1.2 0.3 -0.3 -0.7 0.7 0];
qdotmax=[75 75 90 90 144 135 135];
qdotmax_rad=deg2rad(qdotmax);
max_inc_rad = 0.003;
velocity=1;
control_step_size=0.001;
q_diff=q_goal-q_ini;
qdot_rad=qdotmax_rad*velocity;
qinc_max = qdot_rad*control_step_size;
q_curr=q_ini;
cont=1;
while(cont)
    q_diff=q_goal-q_curr;
    far_ratio=q_diff./qinc_max;
    farest= max(far_ratio);
    if (farest<1)
        q_inc = q_diff;
    else
        q_inc = q_diff./farest;
    end
    q_curr=q_inc+q_curr
    cont = max(abs(q_diff))>0.0001;
end

