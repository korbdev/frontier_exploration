classdef Frontiermap < handle
    properties
        map;
        bins;
        bin_map;
        theta_map;
        theta_list;
        radius_map;
        radius_list;
        frontier_map;
        outer_frontier_map;
        frontiers;
        outer_frontiers;
        frontiers_convex_hull;
        angle_histogram;
        angle_histogram_gauss;
        angle_histogram_normalized_filtered;
        frontier_colors;
        frontier_color_map;
    end
    methods
        function obj = Frontiermap(m, n, map, bins)
            obj.map = map;
            obj.bins = bins;
            obj.bin_map = Inf(m,n);
            obj.theta_map = Inf(m,n);
            obj.radius_map = Inf(m,n);
            obj.frontier_map = zeros(m,n);
            obj.outer_frontier_map = zeros(m,n);
            obj.angle_histogram = zeros(bins,1);
            obj.angle_histogram_normalized_filtered = zeros(bins,1);
            obj.theta_list = [];
            obj.radius_list = [];
            obj.frontiers = [];
            obj.outer_frontiers = [];
            obj.frontiers_convex_hull = [];
            obj.frontier_colors = [];
            obj.frontier_color_map = zeros(m,n);
        end
        function createFrontierMap(obj, pose, sigma, t_h, sm)
            [m, n] = size(obj.map.visibility_map);
            obj.bin_map = Inf(m,n);
            obj.theta_map = Inf(m,n);
            obj.radius_map = Inf(m,n);
            obj.frontier_map = zeros(m,n);
            obj.angle_histogram = zeros(obj.bins,1);
            obj.theta_list = [];
            obj.radius_list = [];
            for i = 2:m-1
                for j = 2:n-1
                    if isFrontier(obj, i , j, sm) == 1
%                        p = transformCoordinatesToRobot([i j], pose);
%                        [theta, radius] = transformCartesianToPolar(p);
%                        deg = floor((theta/(2*pi))*360)+1;
                        
%                        obj.incrementHistogramAt(deg);
                        obj.setFrontier(i,j,0, 0, 0);
                    end
                end
            end
            
            %normalized_hist = normalize(obj.angle_histogram);
            
%            normalized_hist = obj.angle_histogram/(max(obj.angle_histogram));
            
%            t_h_idx = find(normalized_hist < t_h);
%            obj.angle_histogram_normalized_filtered = normalized_hist;
%            obj.angle_histogram_normalized_filtered(t_h_idx) = 0;
            
%            obj.angle_histogram_gauss = gauss(obj.angle_histogram_normalized_filtered, 0, sigma);
            
%            theta_polarplot = zeros(obj.bins, 1);
%            for i = 1:obj.bins
%               theta_polarplot(i) = (i/360)*(2*pi);
%            end
            
            obj.findFrontierSequences(sm);
            
            %obj.findOuterSequences(6, sm);
            
            subplot(2,4,8)%, imshow(frontier_img);
            obj.draw();
%            los_map = obj.map.visibility_map;
%            for i = 1:size(obj.frontiers_convex_hull)-1
%                f =  obj.frontiers_convex_hull(i);
%                for j = (i+1):size(obj.frontiers_convex_hull)
%                    f_n = obj.frontiers_convex_hull(j);
%                    for k = 1:size(f.points,1)
%                        for l = 1:size(f_n.points,1)
%                            los_map = obj.lineofsight(f.points(k,:), f_n.points(l,:), los_map);
%                        end
%                    end 
%                end
%            end
            
            %global color_map;
            %los_img = ind2rgb(los_map, color_map);
            
            %figure(h);
            %subplot(2,4,5)
            %imshow(los_img);
            
            %subplot(2,4,4), polar(theta_polarplot, obj.angle_histogram_gauss), view([0 -90]);
            %subplot(2,3,6), polar(theta_polarplot, obj.angle_histogram_normalized_filtered), view([0 -90]);
        end
        function setFrontier(obj, i, j, bin, theta, radius)
            obj.bin_map(i, j) = bin;
            obj.theta_map(i, j) = theta;
            obj.radius_map(i, j) = radius;
            obj.frontier_map(i, j) = 3;
            obj.theta_list = [obj.theta_list theta];
            obj.radius_list = [obj.radius_list radius];
        end
        function [frontier, bin, theta, radius] = getFrontier(obj, i, j)
            frontier = obj.frontier_map(i, j);
            bin = obj.bin_map(i, j);
            theta = obj.theta_map(i, j);
            radius = obj.radius_map(i, j);
        end
        function incrementHistogramAt(obj, i)
            obj.angle_histogram(i) = obj.angle_histogram(i) + 1;
        end
        function findFrontierSequences(obj, map)
            global color_map;
            next_x = [];
            next_y = [];
            obj.frontiers = [];
            obj.frontiers_convex_hull = [];
            obj.frontier_colors = [0 0 0];
            frontier_count = 1;
            copy_map = obj.frontier_map;
            neighbour_map = obj.frontier_map;
            while any(copy_map(:))
                [r, c] = find(copy_map==3);
                next_x = [next_x, r(1)];
                next_y = [next_y, c(1)];
                
                points = [];
                endpoints = [];
                while any(next_x) && any(next_y)
                    copy_map(next_x(1),next_y(1)) = 0;
                    cnt_neighbours = 3;
                    for i = -1:1
                        for j = -1:1
                            if i~=0 || j~=0
                                xn = next_x(1)+i;
                                yn = next_y(1)+j;
                                %add frontier_point from copy_map to next
                                %list
                                if copy_map(xn,yn) == 3
                                    next_x = [next_x, xn];
                                    next_y = [next_y, yn];
                                    copy_map(xn,yn) = 0;
                                end
                                %check for neighbours in original map
                                if obj.frontier_map(xn,yn) == 3
                                    cnt_neighbours = cnt_neighbours + 1;
                                end
                            end
                        end
                    end
                    
                    %neighbour_map(next_x(1),next_y(1)) = cnt_neighbours;
                    
                    [m,n] = size(next_x);
                    %add frontier to points
                    points = [points; next_x(1), next_y(1);];
                    
                    %point is endpoint, save index
                    if cnt_neighbours == 4
                        endpoints = [endpoints size(points,1)];
                        %neighbour_map(next_x(1), next_y(1)) = 4;
                    end
                    
                    if n > 1
                        next_x = next_x(2:n);
                        next_y = next_y(2:n);
                    else
                        next_x = [];
                        next_y = [];
                    end
                    
                end
                
                %ep = points(endpoints,:);
                
                %neighbour_map(ep(:,1), ep(:,2)) = 4;
                
                %no endpoint found, search for first one
                if size(endpoints,2) == 0 && size(points, 1) > 1
                    to_traverse = points;
                    p = 1;
                    min_dist_idx = 0;
                    while any(to_traverse(:))
                        to_traverse(p,:) = 0;
                        
                        min_dist = Inf;
                        for i = 1:size(to_traverse,1)
                            if any(to_traverse(:))
                                dist = abs(sqrt(sum( (to_traverse(i,:) - points(p,:)).^2)));
                                if dist < min_dist
                                    min_dist = dist;
                                    min_dist_idx = i;
                                end
                            end
                        end
                        p = min_dist_idx;
                    end
                    endpoints = [endpoints min_dist_idx];
                end
                
                if size(endpoints,2) == 1
                    %traverse frontier
                    to_traverse = points;
                    p = endpoints(1);
                    min_dist_idx = 0;
                    while any(to_traverse(:))
                        to_traverse(p,:) = 0;
                        
                        min_dist = Inf;
                        for i = 1:size(to_traverse,1)
                            if any(to_traverse(:))
                                dist = abs(sqrt(sum( (to_traverse(i,:) - points(p,:)).^2)));
                                if dist < min_dist
                                    min_dist = dist;
                                    min_dist_idx = i;
                                end
                            end
                        end
                        p = min_dist_idx;
                    end
                    endpoints = [endpoints min_dist_idx];
                end
                
                %for i = 1:size(endpoints,2)
                %    p = points(endpoints(i),:);
                %    neighbour_map(p(1), p(2)) = 4;
                %end
                
                if size(points, 1) > 1
                    p1 = points(endpoints(1),:);
                    p2 = points(endpoints(2),:);
                    
                    frontier_size = abs(sqrt(sum( (p1 - p2).^2)));
                    %if frontier_size > 6;
                    r = rand();
                    g = rand();
                    b = rand();
                    f = Frontier(frontier_count, points, frontier_count+1, map);
                    f.setEndpoints(endpoints);
                    frontier_count = frontier_count + 1;
                    obj.frontier_colors = [obj.frontier_colors; r g b;];
                    obj.frontiers = [obj.frontiers; f];
                    try
                        k = convhull(points);
                        f_hull = Frontier(points(k,:), frontier_count);
                    catch
                        f_hull = f;
                    end
                    obj.frontiers_convex_hull = [ obj.frontiers_convex_hull; f_hull];
                    %end
                end
            end
            %subplot(2,4,4);
            %neighbour_img = ind2rgb(neighbour_map, color_map);
            %imshow(neighbour_img);
        end
        function findOuterSequences(obj, radius, safety_map)
            obj.outer_frontiers = [];
            temp2_outer_frontiers = [];
            temp_outer_frontiers = [];
            [m, n] = size(safety_map);
            for i = 1:size(obj.frontiers,1)
                frontier = obj.frontiers(i);
                points = [];
                obj.outer_frontier_map = zeros(m,n);
                counter = 0;
                for j = 1:size(frontier.points,1)
                    point = frontier.points(j,:);
                    for k = -radius:radius
                        for l = -radius:radius
                            n_i = point(1)+k;
                            n_j = point(2)+l;
                            %visit only outer bounds and index is in range
                            %if (abs(k) == radius || abs(l) == radius) && ( n_i > 0 && n_i <= m && n_j > 0 && n_j <= n)
                                %counter = counter +1;
                                %fprintf('%d, %d, %d\n', k,l,counter);
                                %if safety_map(n_i, n_j) == 2
                                %    counter = counter +1;
                                %    fprintf('%d, %d, %d\n', n_i,n_j,counter);
                                %   points = [points; n_i n_j];
                                %    obj.frontier_map(n_i, n_j) = 5;
                                %end
                            %end
                            if  n_i > 0 && n_i <= m && n_j > 0 && n_j <= n
                                points = [points; n_i n_j];
                                obj.outer_frontier_map(n_i, n_j) = 5;
                            end
                            %if safety_map
                        end
                    end
                end
                points = unique(points, 'rows');
                %temp_outer_frontiers = [temp_outer_frontiers; Frontier(i, points, 5)];
                points_outer = [];
                for j = 1:size(points,1)
                    p = points(j,:)
                    %counter = counter +1;
                    %fprintf('%d, %d, %d\n', k,l,counter);
                    %f = 0;
                    for k = -1:1
                        for l = -1:1
                            n_i = p(1)+k;
                            n_j = p(2)+l;
                            if k ~= 0 || l ~= 0
                                if n_i > 0 && n_i <= m && n_j > 0 && n_j <= n
                                    if safety_map(n_i, n_j) == 2 && obj.outer_frontier_map(n_i, n_j) == 0
                                        points_outer = [points_outer; p(1), p(2)];
                                        obj.outer_frontier_map(p(1),p(2)) = 4;
                                    end
                                end
                            end
                        end
                    end
                end
                points_outer = unique(points_outer, 'rows');
                obj.outer_frontiers = [obj.outer_frontiers; Frontier(i, points_outer, 4)];

                figure(2)
                imagesc(obj.outer_frontier_map);
            end
            
            
            %{
            for i = 1:size(temp_outer_frontiers,1)
                frontier = temp_outer_frontiers(i);
                points_outer = [];
                for j = 1:size(frontier.points,1)
                    p = frontier.points(j,:)
                    %counter = counter +1;
                    %fprintf('%d, %d, %d\n', k,l,counter);
                    %f = 0;
                    for k = -1:1
                        for l = -1:1
                            n_i = p(1)+k;
                            n_j = p(2)+l;
                            if k ~= 0 || l ~= 0
                                if n_i > 0 && n_i <= m && n_j > 0 && n_j <= n
                                    if safety_map(n_i, n_j) == 2 && obj.outer_frontier_map(n_i, n_j) == 0
                                        points_outer = [points_outer; p(1), p(2)];
                                        obj.outer_frontier_map(p(1),p(2)) = 4;
                                    end
                                end
                            end
                        end
                    end
                end
                points_outer = unique(points_outer, 'rows');
                obj.outer_frontiers = [obj.outer_frontiers; Frontier(i, points_outer, 4)];
            end
            %}

            %{
            for i = 1:size(temp2_outer_frontiers,1)
                frontier = temp2_outer_frontiers(i);
                points = [];
                for j = 1:size(frontier.points,1)
                    p = frontier.points(j,:)
                    %counter = counter +1;
                    %fprintf('%d, %d, %d\n', k,l,counter);
                    %f = 0;
                    for k = -1:1
                        for l = -1:1
                            n_i = p(1)+k;
                            n_j = p(2)+l;
                            if k ~= 0 || l ~= 0
                                if n_i > 0 && n_i <= m && n_j > 0 && n_j <= n
                                    if obj.frontier_map(n_i, n_j) == 0
                                        points = [points; p(1), p(2)];
                                        obj.frontier_map(p(1),p(2)) = 6;
                                    end
                                end
                            end
                        end
                    end
                end
                points = unique(points, 'rows');
                obj.outer_frontiers = [obj.outer_frontiers; Frontier(i, points, 4)];
            end
            %}
        end
        function f = isFrontier(obj, x, y, sm)
            f = 0;
            if sm(x, y) == 3
                if sm(x-1, y) == 2
                    f = 1;
                end
                if sm(x+1, y) == 2
                    f = 1;
                end
                if sm(x, y+1) == 2
                    f = 1;
                end
                if sm(x, y-1) == 2
                    f = 1;
                end
            end
            %{
            if obj.map.visibility_map(x, y) == 3
                if obj.map.visibility_map(x-1, y) == 2
                    f = 1;
                end
                if obj.map.visibility_map(x+1, y) == 2
                    f = 1;
                end
                if obj.map.visibility_map(x, y+1) == 2
                    f = 1;
                end
                if obj.map.visibility_map(x, y-1) == 2
                    f = 1;
                end
            end
            %}
        end
        function map = lineofsight(obj, pose, pose_to, map)
            x = pose(1);
            y = pose(2);
            x_end = pose_to(1);
            y_end = pose_to(2);
            
            dx = x_end - x;
            dy = y_end - y;
            
            %distance = sqrt(dx^2+dy^2);
            
            x_step = 1;
            y_step = 1;
            
            if(dx < 0)
                dx = -dx;
                x_step = -1;
            end
            
            if(dy < 0)
                dy = -dy;
                y_step = -1;
            end
            
            a = 2*dx;
            b = 2*dy;
            
            if(dy <= dx)
                f = -dx;
                map(x,y) = 4;
                while x ~= x_end
                    %{
                    new_dx = x - pose(1);
                    new_dy = y - pose(2);
                    distance = sqrt(new_dx^2+new_dy^2);
                    if(distance < obj.radius)
                        if(obj.occupancy_map(x,y) == obj.wall)
                            map(x,y) = obj.wall;
                            break;
                        else
                            map(x,y) = obj.free;
                        end
                    else
                        break;
                    end
                    %}
                    
                    f = f + b;
                    if(f > 0)
                        y = y + y_step;
                        f = f - a;
                    end
                    x = x + x_step;
                    map(x,y) = 4;
                    %end %
                end
            else
                f = -dy;
                map(x,y) = 4;
                while y ~= y_end
                    %{
                    new_dx = x - pose(1);
                    new_dy = y - pose(2);
                    distance = sqrt(new_dx^2+new_dy^2);
                    if(distance < obj.radius)
                        %break;
                        if(obj.occupancy_map(x,y) == obj.wall)
                            map(x,y) = obj.wall;
                            break;
                        else
                            map(x,y) = obj.free;
                        end
                    else
                        break;
                    end
                    %}
                    
                    f = f + a;
                    if(f > 0)
                        x = x + x_step;
                        f = f - b;
                    end
                    y = y + y_step;
                    map(x,y) = 4;
                    %end %
                end
            end
        end
        function [in, out] = inRange(obj, robot)
            global color_map;
            [m,n] = size(obj.map.visibility_map);
            in = Frontier.empty();
            out = Frontier.empty();
            
           % sensing_map = obj.map.generateSensingMap(robot.pose, robot.sensor.radius+2*robot.robot_size+1);
           sensing_map = obj.map.generateSensingMap(robot.pose, robot.sensor.radius+15);
            for i = 1:size(obj.frontiers,1)
                frontier = obj.frontiers(i);
                sensing_map(frontier.center(1), frontier.center(2)) = 4;
            end
            
            subplot(2, 4, 4);
            sense_img = ind2rgb(sensing_map, color_map);
            imshow(sense_img);
            
            for i = 1:size(obj.frontiers,1)
                frontier = obj.frontiers(i);
                if frontier.getDistance(robot.pose) <= robot.sensor.radius
                    local_planner = PathPlanner(sensing_map, m, n, robot.robot_size+1, true);
                    local_map = local_planner.planCostMap(robot.pose(1), robot.pose(2), false);
                    local_path = local_planner.generatePath(frontier.center(1), frontier.center(2));
                    if ~isempty(local_path)
                        in = [in; frontier];
                    else
                        subplot(2, 4, 3);
                        local_planner.safety_map(frontier.center(1), frontier.center(2)) = 4;
                        sm_img = ind2rgb(local_planner.safety_map, color_map);
                        imshow(sm_img);
                        out = [out; frontier];
                        fprintf('empty\n');
                    end
                else
                    out = [out; frontier];
                end
            end
        end
        function draw(obj)
            global color_map;
            frontier_img = ind2rgb(obj.frontier_map, color_map);
            
            %figure(h);
            imshow(frontier_img);
            
            [m, n] = size(obj.frontier_color_map);
            obj.frontier_color_map = zeros(m,n);
            num_frontiers = size(obj.frontiers, 1);
            x = [];
            y = [];
            k = [];
            
            %h = subplot(2,4,6);
            %cla(h)
            for i=1:num_frontiers
                f = obj.frontiers(i);
                f_h = obj.frontiers_convex_hull(i);
                
                x = f.points(:,2);
                y = f.points(:,1);
                x_h = f_h.points(:,2);
                y_h = f_h.points(:,1);
                %hold on;
                %plot(x, y, 'bx',x_h,y_h,'r-')
                
                for j=1:size(f.points, 1)
                    fp = [f.points(j,1) f.points(j,2)];
                    obj.frontier_color_map(fp(1), fp(2)) = f.color;
                end
                for j= 1:size(f.outer_points,1)
                    fop = [f.outer_points(j,1) f.outer_points(j,2)];
                    obj.frontier_color_map(fop(1), fop(2)) = f.color+1;
                end
            end
            
            %hold off;
            %set(gca,'View',[0 -90]);
            
            subplot(2,4,7)
            frontier_color_img = ind2rgb(obj.frontier_color_map, obj.frontier_colors);
            
            for i = 1:size(obj.frontiers,1)
                s = sprintf('f_%d', obj.frontiers(i).id);
                pos_y = floor(mean(obj.frontiers(i).points(:,1)));
                pos_x = floor(mean(obj.frontiers(i).points(:,2)));
                frontier_color_img = insertText(frontier_color_img,[pos_x pos_y],s);
            end
            imshow(frontier_color_img);
        end
    end
end