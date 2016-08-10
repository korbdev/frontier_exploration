classdef FrontierGraph < handle
    properties
        old_frontiers;
        weights;
        weights_uspace;
        directions;
        dot_product;
        min_dist;
        min_dist_uspace;
        paths;
        paths_uspace;
        planner;
        map;
        collection_uspace;
        collection
    end
    methods
        function obj = FrontierGraph(frontiers, map, planner)
            n = size(frontiers,1);
            obj.old_frontiers = frontiers;
            
            obj.collection_uspace = [];
            obj.collection = [];
            
            obj.weights = Inf(n, n);
            obj.weights(1:n+1:n*n) = 0;
            
            obj.weights_uspace = Inf(n, n);
            obj.weights_uspace(1:n+1:n*n) = 0;
            
            obj.directions = zeros(n, n, 2);
            obj.directions(1:n+1:n*n) = 0;
            
            obj.dot_product = zeros(n, n);
            obj.dot_product(1:n+1:n*n) = -Inf;
            
            obj.min_dist = zeros(n, n);
            obj.min_dist_uspace = zeros(n, n);
            %obj.paths_uspace = zeros(n, n);
            %obj.paths_uspace(1:n+1:n*n) = 0;
            %obj.paths_uspace = [];
            for i = 1:n
                for j = 1:n
                    paths_uspace(i,j) = Path(0);
                end
            end
            obj.paths_uspace = paths_uspace;
            obj.paths_uspace(1:n+1:n*n) = Path(0);
            
            obj.planner = planner;
            obj.map = map;
            %obj.updateWeights();
            obj.updateDijkstraGraph(frontiers);
        end
        function success = exists(obj, frontier)
            success = 0;
            for i = 1:size(obj.frontiers, 1)
                f = obj.map.frontier_map.frontiers(i);
                if isequal(f.points, frontier.points)
                    success = i;
                    return;
                end
            end
        end
        function updateDijkstraGraph(obj, frontiers);
            global color_map;
            drawArrow = @(x,y,varargin) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0, varargin{:} );
            clims = [0 500];
            [m, n] = size(obj.weights);
            %bneck = zeros(240);
            for i = 1:m-1
                from = frontiers(i);
                %from_outer = obj.map.frontier_map.outer_frontiers(i);
                
                planner_kspace = PathPlanner(obj.map.visibility_map, 240, 240, 1, true);
                r_map_kspace = planner_kspace.planCostMap(from.center(1), from.center(2), false);
                
                %subplot(2,4,3);
                %bneck = bneck + planner_kspace.computeBottleNeckMap();
                %imagesc(bneck);
                
                planner_uspace = PathPlanner(obj.map.visibility_map, 240, 240, 1, true);
                r_map_uspace = planner_uspace.planCostMap(from.center(1), from.center(2), true);

                subplot(2, 4, 5);
                imagesc(r_map_kspace, clims);
                
                subplot(2, 4, 6);
                imagesc(r_map_uspace, clims);

                for j = i+1:n
                    to = frontiers(j);
                    %to_outer = obj.map.frontier_map.outer_frontiers(j);
                    if ~isequal(obj.old_frontiers(i).points, frontiers(j).points)
                        %fprintf('Plan Known');
                        path_kspace = planner_kspace.generatePath(to.center(1), to.center(2));
                        %fprintf('Plan Unknown %d %d', i, j);
                        path_uspace = planner_uspace.generatePath(to.center(1), to.center(2));
                        
                        first_point = path_uspace(1, :);
                        second_point = path_uspace(size(path_uspace,1)-1, :);
                        last_point = path_uspace(size(path_uspace,1),:);
                        
                        obj.min_dist(i, j) = sqrt( double((first_point(1)-last_point(1))^2 + (first_point(2)-last_point(2))^2));
                        obj.min_dist(j, i) = sqrt( double((first_point(1)-last_point(1))^2 + (first_point(2)-last_point(2))^2));
                        m_d = [];
                        for num_points = 1:size(path_uspace,1)-1
                            p_1 = path_uspace(num_points,:);
                            p_2 = path_uspace(num_points+1,:);
                            
                            m_d = [m_d ; sqrt(double( (p_1(1)-p_2(1))^2 + (p_1(2)-p_2(2))^2))];
                        end 
                        
                        obj.min_dist_uspace(i, j) = sum(m_d);
                        obj.min_dist_uspace(j, i) = sum(m_d);
                        
                        p_unknown = Path(path_uspace);
                        obj.paths_uspace(i,j) = p_unknown;
                        obj.paths_uspace(j,i) = p_unknown;
                        
                        obj.weights(i,j) = r_map_kspace(to.center(1), to.center(2));
                        obj.weights(j,i) = r_map_kspace(to.center(1), to.center(2));
                        
                        obj.weights_uspace(i,j) = r_map_uspace(to.center(1), to.center(2));
                        obj.weights_uspace(j,i) = r_map_uspace(to.center(1), to.center(2));
                    end
                end
            end
            %%construct direction matrix
            for i = 1:m
                from = frontiers(i);
                subplot(2, 4, 5);
                hold on;
                x1 = [from.center(2) from.center(2)+from.direction(2)*10];
                y1 = [from.center(1) from.center(1)+from.direction(1)*10];

                drawArrow(x1,y1,'linewidth',1,'color','g');
                %hold off;
                for j = 1:m
                    if i ~= j
                        to = frontiers(j);
                        path_uspace = obj.paths_uspace(i,j).waypoints;
                        %if i > j
                        %    path_uspace = obj.paths_uspace(j,i).waypoints;
                        %else
                        %    path_uspace = obj.paths_uspace(i,j).waypoints;
                        %end
                        
                        
                        last_element = from.center;
                        path_direction_sum = 0;
                        if i < j
                            next_element = double(path_uspace(size(path_uspace,1)-1, :));
                        else
                            next_element = double(path_uspace(2, :));
                        end

                        %if i < j
                        %    next_element = double(path_uspace(size(path_uspace,1)-1, :));
                        %else
                        %    next_element = double(path_uspace(2, :));
                        %end
                        
                        %for k=2:size(path_uspace,1)-1
                        %    path_single_direction = next_element - last_element;
                        %    path_direction_sum = path_direction_sum + path_single_direction;
                        %    last_element = next_element;
                        %    k
                        %    if i < j
                        %        next_element = double(path_uspace(size(path_uspace,1)-k, :));
                        %    else
                        %        next_element = double(path_uspace(k+1, :));
                        %    end
                        %end
                        path_single_direction = next_element - last_element;
                        path_direction_sum = path_direction_sum + path_single_direction;
                        
                        %subplot(2, 4, 5);
                        %hold on;
                        path_direction = path_direction_sum/2;
                        x1 = [from.center(2) from.center(2)+path_direction(2)*10];
                        y1 = [from.center(1) from.center(1)+path_direction(1)*10];

                        drawArrow(x1,y1,'linewidth',1,'color','r');
                        %hold off;
                        obj.directions(i,j,1) = path_direction(1);
                        obj.directions(i,j,2) = path_direction(2);
                        dp = dot(from.direction, path_direction);
                        obj.dot_product(i,j) = dp;
                    end
                end
            end
        end
        function updateGraph(obj, frontiers)
            to_keep = [];
            
            for i = 1:size(obj.old_frontiers,1)
                keep = 0;
                for j = 1:size(frontiers,1)
                    if isequal(obj.old_frontiers(i).points, frontiers(j).points)
                        keep = i;
                    end
                end
                if keep ~= 0
                    to_keep = [to_keep, keep];
                end
            end
            
            obj.weights
            to_keep
            obj.weights = obj.weights([to_keep],[to_keep]);
            obj.weights_uspace = obj.weights_uspace([to_keep],[to_keep]);
            obj.paths_uspace = obj.paths_uspace([to_keep],[to_keep]);
            
            to_add = [];
            for i = 1:size(frontiers,1)
                add = i;
                for j = 1:size(obj.old_frontiers,1)
                    if isequal(frontiers(i).points, obj.old_frontiers(j).points)
                        add = 0;
                    end
                end
                if add ~= 0
                    to_add = [to_add, add];
                end
            end
            obj.weights = cat(1, obj.weights, Inf(size(to_add, 2),size(obj.weights,1)));
            obj.weights = cat(2, obj.weights, Inf(size(obj.weights,1),size(to_add, 2)));
            n = size(obj.weights,1);
            obj.weights(1:n+1:n*n) = 0;
            
            obj.weights_uspace = cat(1, obj.weights_uspace, Inf(size(to_add, 2),size(obj.weights_uspace,1)));
            obj.weights_uspace = cat(2, obj.weights_uspace, Inf(size(obj.weights_uspace,1),size(to_add, 2)));
            n = size(obj.weights_uspace,1);
            obj.weights_uspace(1:n+1:n*n) = 0;
            
            
            if size(to_add, 2) > 0 && size(obj.paths_uspace,1) > 0
                t_m(size(to_add, 2),size(obj.paths_uspace,1)) = Path(0);
            else
                t_m = [];
            end
            
            obj.paths_uspace = cat(1, obj.paths_uspace, t_m)
            
            if size(to_add, 2) > 0 && size(obj.paths_uspace,1) > 0
                t_n(size(obj.paths_uspace, 1),size(to_add,2)) = Path(0);
            else
                t_n = [];
            end
            obj.paths_uspace = cat(2, obj.paths_uspace, t_n)
            
            n = size(obj.paths_uspace,1);
            obj.paths_uspace(1:n+1:n*n) = Path(0);
            
            sorted_frontiers = Frontier.empty;
            %for i = 1:size(frontiers,1)
            %   index = frontiers(i).id;
            %    sorted_frontiers(index) = frontiers(i);
            %end
            f_counter = 1;
            for i = 1:size(to_keep,2)
                idx = to_keep(i);
                f = obj.old_frontiers(idx);
                f.id = f_counter;
                f_counter = f_counter + 1;
                sorted_frontiers = [sorted_frontiers; f];
            end
            
            for i = 1:size(to_add,2)
                idx = to_add(i);
                f = frontiers(idx);
                f.id = f_counter;
                f_counter = f_counter + 1;
                sorted_frontiers = [sorted_frontiers; f];
            end
            obj.old_frontiers = obj.map.frontier_map.frontiers;
            obj.map.frontier_map.frontiers = sorted_frontiers;
            obj.updateWeights();
        end
        function updateWeights(obj)
            global color_map;
            [m,n] = size(obj.weights);
            obj.weights
            
            temp_map = obj.map.visibility_map;
            
            for i = 1:m-1
                for j = i+1:n
                    if isinf(obj.weights(i,j))
                        if obj.map.frontier_map.frontiers(i).id ~= i
                            sprintf('%d, %d',obj.map.frontier_map.frontiers(i).id,i);
                        end
                        if obj.map.frontier_map.frontiers(j).id ~= j
                            sprintf('%d, %d',obj.map.frontier_map.frontiers(j).id,j);
                        end
                        %f_start = obj.map.frontier_map.frontiers(i).points;
                        %f_goal = obj.map.frontier_map.frontiers(j).points;
                        
                        %start = [floor(mean(f_start(:,1))) floor(mean(f_start(:,2)))];
                        %finish = [floor(mean(f_goal(:,1))) floor(mean(f_goal(:,2)))];
                        
                        start = obj.map.frontier_map.frontiers(i).center;
                        finish = obj.map.frontier_map.frontiers(j).center;
                        
                        %find collission free frontier point
                        for y = -2:2
                            for x = -2:2
                                s_i = start(1)+y;
                                s_j = start(2)+x;
                                start_collision = obj.planner.checkSquareCollision(obj.map.visibility_map, [s_i s_j],2*2, 0);
                                
                                temp_map(s_i, s_j) = 5;
                                
                                if start_collision == 0
                                    new_start = [s_i s_j];
                                end
                            end
                        end
                        
                        
                        for y = -2:2
                            for x = -2:2
                                f_i = finish(1)+y;
                                f_j = finish(2)+x;
                                finish_collision = obj.planner.checkSquareCollision(obj.map.visibility_map, [f_i f_j],2*2, 0);
                                
                                temp_map(f_i, f_j) = 6;
                                
                                if finish_collision == 0
                                    new_finish = [f_i f_j] ;
                                end
                            end
                        end
                        points_known = obj.planner.plan(new_start, new_finish, obj.map.visibility_map, 4, 1, 0);
                        
                        %find collission free frontier point
                        for y = -4:4
                            for x = -4:4
                                s_i = start(1)+y;
                                s_j = start(2)+x;
                                start_collision = obj.planner.checkSquareCollision(obj.map.visibility_map, [s_i s_j],2*2, 1);
                                
                                temp_map(s_i, s_j) = 5;
                                
                                if start_collision == 0
                                    new_start = [s_i s_j];
                                    temp_map(new_start(1), new_start(2)) = 6;
                                end
                            end
                        end
                        
                        for y = -4:4
                            for x = -4:4
                                f_i = finish(1)+y;
                                f_j = finish(2)+x;
                                finish_collision = obj.planner.checkSquareCollision(obj.map.visibility_map, [f_i f_j],2*2, 1);
                                
                                temp_map(f_i, f_j) = 6;
                                
                                if finish_collision == 0
                                    new_finish = [f_i f_j] ;
                                    temp_map(new_finish(1), new_finish(2)) = 6;
                                end
                            end
                        end
                        points_unknown = obj.planner.plan(new_start, new_finish, obj.map.visibility_map, 4, 1, 1);
                        
                        subplot(2,4,4)
                        temp_img = ind2rgb(temp_map, color_map);
                        imshow(temp_img);
                        
                        p_unknown = Path(points_unknown);
                        obj.paths_uspace(i,j) = p_unknown;
                        obj.paths_uspace(j,i) = p_unknown;
                        if size(points_known,1) == 0
                            length_kspace = Inf;
                        else
                            length_kspace = size(points_known,1);
                        end
                        if size(points_unknown,1) == 0
                            length_uspace = Inf;
                        else
                            length_uspace = size(points_unknown,1);
                            
                            %if path found, calculate distances
                            first_point = points_unknown(1, :);
                            last_point = points_unknown(size(points_unknown,1),:);
                            obj.min_dist(i, j) = sqrt( (first_point(1)-last_point(1))^2 + (first_point(2)-last_point(2))^2);
                            
                            m_d = [];
                            for num_points = 1:size(points_unknown,1)-1
                                p_1 = points_unknown(num_points,:);
                                p_2 = points_unknown(num_points+1,:);
                                
                                m_d = [m_d ; sqrt( (p_1(1)-p_2(1))^2 + (p_1(2)-p_2(2))^2)];
                            end
                            obj.min_dist_uspace(i, j) = sum(m_d);
                        end
                        obj.weights(i,j) = length_kspace;
                        obj.weights_uspace(i,j) = length_uspace;
                    end
                end
            end
            
            obj.weights = triu(obj.weights)'+triu(obj.weights);
            obj.weights_uspace = triu(obj.weights_uspace)'+triu(obj.weights_uspace);
            obj.min_dist = triu(obj.min_dist)'+triu(obj.min_dist);
            obj.min_dist_uspace = triu(obj.min_dist_uspace)'+triu(obj.min_dist_uspace);
            
            Diff = obj.weights_uspace < obj.weights
            
            r_1 = floor((1-size(obj.weights,2)).*rand(1,1) + size(obj.weights,2))
            r_2 = floor((1-size(obj.weights,2)).*rand(1,1) + size(obj.weights,2))
            obj.planner.planGraph(obj.weights_uspace, r_1, r_2);
            %obj.collection = [obj.collection obj.weights];
            %obj.collection_uspace = [obj.collection_uspace obj.weights_uspace]
        end
    end
end