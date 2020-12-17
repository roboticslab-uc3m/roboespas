%CAMBIAR HERRAMIENTA

init_ros;
IiwaCommandStack.GravityCompensationMode('cartesian', 'YZ');
waitfor(msgbox('Mueva el robot hasta el primer punto'));

P1=IiwaCommandStack.ReadCurrentJointPosition();
waitfor(msgbox('Mueva el robot hasta el segundo punto'));

P2=IiwaCommandStack.ReadCurrentJointPosition();

waitfor(msgbox('Mueva el robot hasta el tercer punto'));
P3=IiwaCommandStack.ReadCurrentJointPosition();

IiwaCommandStack.GravityCompensationMode('stop');

pause(0.2);
IiwaCommandStack.SetVelocity(0.2);
pause(0.2);
[~, traj_out1] = IiwaCommandStack.MoveJ(P1);
[~, traj_out2] = IiwaCommandStack.MoveJ(P2);
[~, traj_out3] = IiwaCommandStack.MoveJ(P3);

f = figure;
IiwaPlotter.cartesian_positions3d({traj_out1, traj_out2, traj_out3}, {'b', 'r', 'g'}, f);



