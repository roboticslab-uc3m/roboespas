clear all
qini=[0 0 0 0 0 0 0];
qend=[1.2 -1.2 0.3 -0.3 -0.7 0.7 0];
vmax=[75 75 90 90 144 135 135];
vmax_rad=deg2rad(vmax);
max_inc_rad = 0.003;
velocity=1;


v_rad=vmax_rad*velocity;
incq=qend-qini;
tmin=abs(incq)./v_rad;
ttotal=max(tmin)
%Get sample_step_size for this time to fullfil the condition of max_inc_rad
max_sample_step_size = max_inc_rad*ttotal / max(incq)
sample_step_size=floor(max_sample_step_size*1000)/1000

samples=ceil(ttotal/sample_step_size);

for i=1:samples+1
    q(:,i)=qini+incq/samples*(i-1);
    if (i>1)
        incqi(:,i)=q(:,i)-q(:,i-1);
    end
end

t=0:sample_step_size:sample_step_size*samples;
size(t)

traj=IiwaTrajectory();
traj.q=q';
traj.t=t;