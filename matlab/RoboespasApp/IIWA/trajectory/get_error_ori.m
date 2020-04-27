function [error_ori, t_error] = get_error_ori(data_in, data_out, t_in, t_out)
% Standard function used for calculating the error between different
% arrays. In this case data_in and data_out are not structures, but the
% field in which the error is being calculated. It builds timeseries for
% input and output, synchronize them, and then calculate the error using
% the function substract_ori, which transforms euler angles into rotation
% matrices, operate with them, and then return a set of euler angles.
    ts_input=timeseries(data_in, t_in);
    ts_output=timeseries(data_out, t_out);
    [ts_input, ts_output]=synchronize(ts_input, ts_output, 'union');
    for i=1:size(ts_output.Data(:,:),2)
        error_ori(:, i)=substract_ori(ts_output.Data(:,i)', ts_input.Data(:,i)', 1);
        if any(abs(rad2deg(error_ori(:,i)))>10)
            error_ori(:,i)=error_ori(:,i-1);
        end
    end
    t_error=ts_input.Time;
end

