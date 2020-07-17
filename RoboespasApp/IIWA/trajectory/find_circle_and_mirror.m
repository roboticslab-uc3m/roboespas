function [data_output, ang_vel] = find_circle_and_mirror(data_input, tpause, smooth_adjusted, bool_plot, flexion, fixed_plane, vel_default)
    %vel_default=30;
    %Avoid initial and ending stops to correctly get circle
    data_input=delete_pause_init_end(data_input);
    %Parameters
    npoints=20; %6 points to find the circumference
%     fixed_plane=1; %Fix X plane

    %Load points used for finding the circle
    if flexion
        points=data_input.x.pos';
    else % si el movimiento grabado es la extensión, hacer un flip de los datos para que las repeticiones empiecen siempre por la flexión
        points = fliplr(data_input.x.pos)';
    end
    points_used=points(1:round(size(data_input.x.pos,2)/npoints):end,:);

    coords_used=[1 2 3];
    coords_used(fixed_plane)=[];
    points_plane=points_used(:,coords_used);

    %% Plot points
    if (bool_plot)
        figure;
        hold on;
        axis equal;
        plot(points_plane(:,1), points_plane(:,2), 'b.');
    end
    %% Find segments and perpendicular lines
    margin=0.10;
    xmin=min(points_plane(:,1))-margin;
    xmax=max(points_plane(:,1))+margin;
    ymin=min(points_plane(:,2))-margin;
    ymax=max(points_plane(:,2))+margin;
    x=linspace(xmin, xmax, 1000);
    for i=1:size(points_plane,1)-1
        %Plot segment
        seg.x=[points_plane(i,1), points_plane(i+1,1)];
        seg.y=[points_plane(i,2), points_plane(i+1,2)];
        B=[ones(size(seg.x')) seg.x']\seg.y';
        seg.intercept=B(1);
        seg.slope=B(2);
        seg.center.x=mean(seg.x);
        seg.center.y=mean(seg.y);
        if (bool_plot)
            plot(seg.x, seg.y);
        end

        %Find perpendicular line between (xmin, xmax)
        y=-1/seg.slope*(x-seg.center.x)+seg.center.y;
        in_axis=(max(find(y>=ymin & y<=ymax, 1, 'first')-1,1):min(find(y>=ymin & y<=ymax, 1, 'last')+1,size(y,2)));
        if (isempty(in_axis))
            %in_axis=(max(find(x>=xmin & x<=xmax, 1, 'first')-1,1):min(find(x>=xmin & x<=xmax, 1, 'last')+1,size(x,2)));
            ME=MException('Adjust:TODO', 'ToDo: Treat with horizontal lines');
            throw(ME);
        end
        xx=x(in_axis);
        yy=y(in_axis);
        perp.x=[xx(1), xx(end)];
        perp.y=[yy(1), yy(end)];
        B=[ones(size(perp.x')) perp.x']\perp.y';
        perp.intercept=B(1);
        perp.slope=B(2);
        if (bool_plot)
            plot(perp.x, perp.y);
        end
        segs(i)=seg;
        perps(i)=perp;
    end

    %% Find intersections
    intersections=[];
    for i=1:size(perps,2)
        for j=1:size(perps,2)
            if (i~=j)
                [x,y]=polyxpoly(perps(i).x, perps(i).y, perps(j).x, perps(j).y);
                intersections=[intersections; x y];
            end
        end
    end
    if (bool_plot)
        plot(intersections(:,1), intersections(:,2), 'x');
    end

    %% Find center and radius of circumference
    C=mean(intersections);
    if (bool_plot)
        plot(C(1), C(2), 'X', 'MarkerSize', 10);
    end

    %% Find radius of circumference
    R=mean(vecnorm((points_plane-C)'));

    %% Find initial and ending points of arc
    %1. Find lines that join center with initial and ending points
    line_p1_center.x=[points(1, coords_used(1)), C(1)];
    line_p1_center.y=[points(1, coords_used(2)), C(2)];
    B=[ones(size(line_p1_center.x')) line_p1_center.x']\line_p1_center.y';
    line_p1_center.intercept=B(1);
    line_p1_center.slope=B(2);
    
    line_pend_center.x=[points(end, coords_used(1)), C(1)];
    line_pend_center.y=[points(end, coords_used(2)), C(2)];
    B=[ones(size(line_pend_center.x')) line_pend_center.x']\line_pend_center.y';
    line_pend_center.intercept=B(1);
    line_pend_center.slope=B(2);
    
    %2. Find intersections of those lines with calculated circumference (2
    %points intersecting each of 2 lines)
    [x1, y1]=linecirc(line_p1_center.slope, line_p1_center.intercept, C(1), C(2), R);
    xy1=[x1; y1];
    
    [xend, yend]=linecirc(line_pend_center.slope, line_pend_center.intercept, C(1), C(2), R);
    xyend=[xend; yend];
    
    %3. Select intersection point close to given points
    d1=vecnorm(xy1-points(1,coords_used)');
    [~, id1]=min(d1);
    xy1=xy1(:,id1)';
    
    dend=vecnorm(xyend-points(end,coords_used)');
    [~, idend]=min(dend);
    xyend=xyend(:,idend)';
    
    %% Find circumferences arc 
    A = [xy1(1); xy1(2)];
    B = [xyend(1); xyend(2)]; % Same with point B
    a = atan2(A(2)-C(2),A(1)-C(1));
    b = atan2(B(2)-C(2),B(1)-C(1));
    b = mod(b-a,2*pi)+a; % Ensure that arc moves counterclockwise
    t = linspace(a,b,size(data_input.q,2));
    x = C(1)+R*cos(t);
    y = C(2)+R*sin(t);
    if (bool_plot)
        plot(x,y, 'LineWidth', 3)
    end
    
    points_out(:,coords_used)=[x;y]';
    points_out(:,fixed_plane)=mean(points(:,1));

    %% Find circumference angle and velocity
    
    u=[0 xy1-C];
    v=[0 xyend-C];
    angle_arc = rad2deg(atan2(norm(cross(u,v)), dot(u,v)));
    ang_vel=angle_arc/data_input.t(end);
    
    %% Build data structure
    data_circle.x.pos=points_out';
    data_circle.x.ori=data_input.x.ori;
    data_circle.t=linspace(0,data_input.t(end),size(data_input.t,2)); %iguala la velocidad porque los puntos están equidistanciados
    %% Add pause
    tsample_mean=mean(data_circle.t(2:end)-data_circle.t(1:end-1));
    added_stamps_pause = zeros(1, round(tpause/tsample_mean));
    added_stamps_pause(1)=data_circle.t(end)+tsample_mean;
    for i=2:round(tpause/tsample_mean)
        added_stamps_pause(i)=added_stamps_pause(i-1)+tsample_mean;
    end
    added_xpos_pause=ones(3,size(added_stamps_pause,2)).*data_circle.x.pos(:,end);
    added_xori_pause=ones(3,size(added_stamps_pause,2)).*data_circle.x.ori(:,end);
    data_withpause.x.pos=[data_circle.x.pos added_xpos_pause];
    data_withpause.x.ori=[data_circle.x.ori added_xori_pause];
    data_withpause.t=[data_circle.t added_stamps_pause];
    
    diff_stamps=data_circle.t(2:end)-data_circle.t(1:end-1);
    diff_stamps_flipped=fliplr(diff_stamps);
    added_stamps_mirror = zeros(1, size(diff_stamps_flipped,2)+1);
    added_stamps_mirror(1)=data_withpause.t(end);
    for i=2:size(diff_stamps_flipped,2)+1
        added_stamps_mirror(i)=added_stamps_mirror(i-1)+diff_stamps_flipped(i-1);
    end
    data_mirrored.t=[data_withpause.t added_stamps_mirror(2:end)];
    added_xpos_mirror=fliplr(data_circle.x.pos);
    added_xori_mirror=fliplr(data_circle.x.ori);
    data_mirrored.x.pos=[data_withpause.x.pos added_xpos_mirror(:,2:end)];
    data_mirrored.x.ori=[data_withpause.x.ori added_xori_mirror(:,2:end)];
    
    %% Build q data
    displacement=data_mirrored.x.pos(:,1)-data_input.x.pos(:,1);
    displacement(4:6)=[0 0 0];
    q_ini=IDK_point_straightline(data_input.q(:,1), displacement');
    data_q=IDK_trajectory(data_mirrored.x.pos, data_mirrored.x.ori, data_mirrored.t, q_ini);
    
    data_output=bounded_spline(data_q, 1, smooth_adjusted); %%%TODO: Add velocity effect
end

