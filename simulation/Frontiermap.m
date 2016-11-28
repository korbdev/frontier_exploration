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
                        obj.setFrontier(i,j,0, 0, 0);
                    end
                end
            end
            
            obj.findFrontierSequences(sm);
            
            subplot(2,4,8);
            obj.draw();
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
                end
            end
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
        end
        function [in, out] = inRange(obj, robot)
            global color_map;
            global calc_counter;
            [m,n] = size(obj.map.visibility_map);
            in = Frontier.empty();
            out = Frontier.empty();
            
            sensing_map = obj.map.generateSensingMap(robot.pose, robot.sensor.radius+robot.robot_size+1);
            
            for i = 1:size(obj.frontiers,1)
                frontier = obj.frontiers(i);
                sensing_map(frontier.center(1), frontier.center(2)) = 4;
            end

            local_planner = PathPlanner(sensing_map, m, n, robot.robot_size+1, true);
            local_map = local_planner.planCostMap(robot.pose(1), robot.pose(2), false);
            
            for i = 1:size(obj.frontiers,1)
                frontier = obj.frontiers(i);
                if frontier.getDistance(robot.pose) <= robot.sensor.radius
                    
                    local_path = local_planner.generatePath(frontier.center(1), frontier.center(2));
                    if ~isempty(local_path)
                        in = [in; frontier];
                    else
                        out = [out; frontier];
                        fprintf('empty\n');
                    end
                else
                    out = [out; frontier];
                end
            end
        end
        function id = getFrontierId(obj, x, y)
            id = 0;
            for i = 1:size(obj.frontiers,1)
                frontier = obj.frontiers(i);
                frontier.center
                xy = [x y]
                if frontier.center(1) == x && frontier.center(2) == y
                    id = frontier.id
                end
            end
        end
        function draw(obj)
            global color_map;
            global calc_counter;
            frontier_img = ind2rgb(obj.frontier_map, color_map);
            
            %figure(h);
            imshow(frontier_img);
            
            frontier_map_s = sprintf('~/research/frontier_exploration/simulation/frontier_map/frontier_map_%d.png', calc_counter);
            imwrite(frontier_img, frontier_map_s);
            
            
            [m, n] = size(obj.frontier_color_map);
            obj.frontier_color_map = zeros(m,n);
            num_frontiers = size(obj.frontiers, 1);
            x = [];
            y = [];
            k = [];

            for i=1:num_frontiers
                f = obj.frontiers(i);
                f_h = obj.frontiers_convex_hull(i);
                
                x = f.points(:,2);
                y = f.points(:,1);
                x_h = f_h.points(:,2);
                y_h = f_h.points(:,1);
                
                for j=1:size(f.points, 1)
                    fp = [f.points(j,1) f.points(j,2)];
                    obj.frontier_color_map(fp(1), fp(2)) = f.color;
                end
                for j= 1:size(f.outer_points,1)
                    fop = [f.outer_points(j,1) f.outer_points(j,2)];
                    obj.frontier_color_map(fop(1), fop(2)) = f.color+1;
                end
            end
            
            subplot(2,4,7)
            frontier_color_img = ind2rgb(obj.frontier_color_map, obj.frontier_colors);
            
            for i = 1:size(obj.frontiers,1)
                s = sprintf('f_%d', obj.frontiers(i).id);
                pos_y = floor(mean(obj.frontiers(i).points(:,1)));
                pos_x = floor(mean(obj.frontiers(i).points(:,2)));
                frontier_color_img = insertText(frontier_color_img,[pos_x pos_y],s);
            end
            
            frontier_map_id = sprintf('~/research/frontier_exploration/simulation/frontier_map_id/frontier_map_id_%d.png', calc_counter);
            imwrite(frontier_color_img, frontier_map_id);
            
            imshow(frontier_color_img);
        end
    end
end