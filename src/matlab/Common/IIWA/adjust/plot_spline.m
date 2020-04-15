function plot_spline(njoint, t, coefs, color)
% Function that plots a spline with the points between segments marked as
% points.
    for i=1:size(t,2)-1
        tt_local=linspace(0, t(i+1)-t(i));
        poly_val=polyval(coefs{njoint}(:,i), tt_local);
        tt=linspace(t(i), t(i+1));
        plot(tt, rad2deg(poly_val), color, 'LineWidth', 2);
        plot([tt(1), tt(end)], rad2deg([poly_val(1), poly_val(end)]), 'k.', 'MarkerSize', 5);
    end
end

