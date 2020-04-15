function [error, t_error] = get_error(data_in, data_out, t_in, t_out)
% Standard function used for calculating the error between different
% arrays. In this case data_in and data_out are not structures, but the
% field in which the error is being calculated. It builds timeseries for
% input and output, synchronize them, and then calculate the error by
% substracting contiguous data points and dividing by the difference
% between contiguous timestamps.
    ts_input=timeseries(data_in, t_in);
    ts_output=timeseries(data_out, t_out);
    [ts_input, ts_output]=synchronize(ts_input, ts_output, 'union');
    error=ts_output.Data(:,:)-ts_input.Data(:,:);
    t_error=ts_input.Time;
end