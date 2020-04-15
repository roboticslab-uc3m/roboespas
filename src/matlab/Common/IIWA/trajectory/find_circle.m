function [data_circle] = find_circle(data_input, varargin)
    %Avoid initial and ending stops to correctly get circle
    data_input=delete_pause_init_end(data_input);
    %Parameters
    bool_plot=0;
    if size(varargin)~=0
        bool_plot=varargin{1};
    end
    npoints=20; %6 points to find the circumference
    fixed_plane=1; %Fix X plane

    %Load points used for finding the circle
    points=data_input.x.pos';
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
    %% Build output data
    displacement=points_out(1,:)'-data_input.x.pos(:,1);
    displacement(4:6)=[0 0 0];
    q1=IDK_point_straightline(data_input.q(:,1), displacement');
    t=linspace(0,data_input.t(end),size(data_input.t,2));
    data_circle=IDK_trajectory(points_out', data_input.x.ori, t, q1);
end

