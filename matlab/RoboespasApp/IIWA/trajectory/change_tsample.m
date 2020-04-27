function data_output = change_tsample(data, tsample)
% Function that changes the time sample of a data struct
    % Init data_output
    data_output=data;
    % Find new time stamps vector
    tnew = 0:tsample:data.t(end);
    tnew = [tnew, data.t(end)]; %Add last time_stamp;
    data_output.t=tnew;
    % Find q for that time stamps
    data_output = get_data_from_pp(data_output);
    data.tsample=tsample;
end

