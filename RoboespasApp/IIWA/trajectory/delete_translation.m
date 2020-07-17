function [data_input, data_notranslation, newname] = delete_translation(name, folder, q_execution)
    newname=strcat(name, '_notranslation');
    data_input=capture_load(name, folder);
    q_out=IDK_point_straightline(data_input.q(:,1), displacement);
    data_no_translation=IDK_trajectory(data_input.x.pos+displacement(1:3)', data_input.x.ori+displacement(4:6)', data_input.t, q_out);
    data_displaced_spline=bounded_spline(data_no_translation,1);
    capture_new(newname, folder, data_displaced_spline);
end

