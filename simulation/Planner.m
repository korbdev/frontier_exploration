classdef Planner < handle
    %PLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot;
    end
    
    methods
        function obj = Planner(robot)
            obj.robot = robot;
        end
        function [r_map, path_map] = planCostMap(obj, map, pose,throughUnknown)
            [m, n] = size(map);
            [r_map, path_map] = dijkstra(map, m, n, pose(2), pose(1), throughUnknown);
        end
        function r_map = planReachabilityMap(obj, start, map, throughUnknown)
            pQ = java.util.PriorityQueue;
            [m, n] = size(map);
            %Q = Inf(m,n);
            r_map = Inf(m,n);
            %Q(start(1), start(2)) = 0;
            r_map(sub2ind(size(r_map), start(1), start(2))) = 0;
            node = GridNode(0, sub2ind(size(r_map), start(1), start(2)));
            
            pQ.add(node);
            loop = 0;
            while ~pQ.isEmpty()
                
                loop = loop +1;
                I = pQ.poll();
                d = I.getKey();
                v = I.getValue();
                [i, j] = ind2sub(size(r_map), v);
                %Q(i, j) = Inf;
                for k = -1:1
                    for l = -1:1
                        if(i+k > 0 && i+k < m+1 && j+l > 0 && j+l < n+1)
                            if(k~=0 || l~=0 )
                                if map(i+k, j+l) ~= 1
                                    if isinf(r_map(i+k, j+l))
                                        new_dist = r_map(i,j) + sqrt( (i-(i+k))*(i-(i+k)) + (j-(j+l))*(j-(j+l)));
                                       
                                        pQ.add( GridNode(new_dist, sub2ind(size(r_map), i+k, j+l)));
                                        %r_map(i+k, j+l) = new_dist;
                                        %Q(i+k, j+l) = r_map(i+k, j+l);
                                        if new_dist < r_map(i+k, j+l)
                                            r_map(i+k, j+l) = new_dist;
                                            %Q(i+k, j+l) = r_map(i+k, j+l);
                                        end
                                    end
                                    %if ~isinf(Q(i,j))
                                    %    if new_dist < r_map(i+k, j+l)
                                    %        r_map(i+k, j+l) = new_dist;
                                    %        Q(i+k, j+l) = r_map(i+k, j+l);
                                    %    end
                                    %end
                                end
                            end
                        end
                    end
                end
            end
        end
        function path = plan(obj, start, goal, map, buffer, test_collision, unknown)
            path = [];
            [m,n] = size(map);
            parent = ones(m,n);
            open_set = Inf(m,n);
            closed_set = Inf(m,n);
            %f_score = zeros(m,n);
            g_score = zeros(m,n);
            
            open_set(start(1), start(2)) = 0;
            
            while find(isfinite(open_set))
                [M, I] = min(open_set(:));
                [i, j] = ind2sub(size(open_set), I);
                open_set(i, j) = Inf;
                closed_set(i,j) = 1;
                if [i,j] == goal
                    break;
                end
                for k = -1:1
                    for l = -1:1
                        if(i+k > 0 && i+k < m && j+l > 0 && j+l < n) %in range
                            if(k~=0 || l~=0 )
                                cost = g_score(i,j)+1;
                                n_i = i+k;
                                n_j = j+l;
                                collision = 0;
                                if test_collision == 1
                                    %collision = obj.checkCollision(map, [n_i n_j],buffer, unknown);
                                    collision = obj.checkSquareCollision(map, [n_i n_j],buffer, unknown);
                                end
                                if obj.isFeasibleTerrain(map(n_i, n_j), unknown) && ~collision % && ~obj.checkCollision(map, [n_i n_j],buffer))
                                    if ~isinf(open_set(n_i, n_j)) && cost < g_score(n_i, n_j)
                                        open_set(n_i, n_j) = Inf;
                                    end
                                    if ~isinf(closed_set(n_i, n_j)) && cost < g_score(n_i, n_j)
                                        closed_set(n_i, n_j) = Inf;
                                    end
                                    if isinf(open_set(n_i, n_j)) && isinf(closed_set(n_i, n_j))
                                        g_score(n_i, n_j) = cost;
                                        f_score = cost + obj.heuristic([n_i n_j],goal);
                                        open_set(n_i, n_j) = f_score;
                                        parent(n_i, n_j) = i * n +j;
                                    end
                                end
                            end
                        end
                    end
                end
            end
            
            position = goal;
            
            while(~isequal(start, position))
                index = parent(position(1), position(2));
                if index == 1
                    break;
                end
                i = floor(index / n);
                j = mod(index, n);
                path = [path; i j];
                position = [i j];
            end
        end
        function path = plan2(obj, start, goal, map, buffer, test_collision, unknown)
            path = [];
            [m,n] = size(map);
            parent = ones(m,n);
            open_set = Inf(m,n);
            closed_set = Inf(m,n);
            %f_score = zeros(m,n);
            g_score = zeros(m,n);
            
            open_set(start(1), start(2)) = 0;
            
            while find(isfinite(open_set))
                [M, I] = min(open_set(:));
                [i, j] = ind2sub(size(open_set), I);
                open_set(i, j) = Inf;
                closed_set(i,j) = 1;
                if [i,j] == goal
                    break;
                end
                for k = -1:1
                    for l = -1:1
                        if(i+k > 3 && i+k < m-3 && j+l > 3 && j+l < n-3) %in range
                            if(k~=0 || l~=0 )
                                cost = g_score(i,j)+1;
                                n_i = i+k;
                                n_j = j+l;
                                collision = 0;
                                if test_collision == 1
                                    collision = obj.checkCollision(map, [n_i n_j],buffer,unknown);
                                end
                                if obj.isFeasibleTerrain(map(n_i, n_j), unknown) && ~collision % && ~obj.checkCollision(map, [n_i n_j],buffer))
                                    if g_score(n_i, n_j) == 0 || cost < g_score(n_i, n_j)
                                        g_score(n_i, n_j) = cost;
                                        f_score = cost + obj.heuristic([n_i n_j],goal);
                                        open_set(n_i, n_j) = f_score;
                                        parent(n_i, n_j) = i * n +j;
                                    end
                                    %{
                                    if ~isinf(open_set(n_i, n_j)) && cost < g_score(n_i, n_j)
                                        open_set(n_i, n_j) = Inf;
                                    end
                                    if ~isinf(closed_set(n_i, n_j)) && cost < g_score(n_i, n_j)
                                        closed_set(n_i, n_j) = Inf;
                                    end
                                    if isinf(open_set(n_i, n_j)) && isinf(closed_set(n_i, n_j))
                                        g_score(n_i, n_j) = cost;
                                        f_score = cost + obj.heuristic([n_i n_j],goal);
                                        open_set(n_i, n_j) = f_score;
                                        parent(n_i, n_j) = i * n +j;
                                    end
                                    %}
                                end
                            end
                        end
                    end
                end
            end
            
            position = goal;
            
            while(~isequal(start, position))
                index = parent(position(1), position(2));
                if index == 1
                    break;
                end
                i = floor(index / n);
                j = mod(index, n);
                path = [path; i j];
                position = [i j];
            end
        end
        function path = planGraph(obj, graph, start, goal)
            path = [];
            n = size(graph,2);
            parent = zeros(n,1);
            open_set = Inf(n,1);
            closed_set = Inf(n,1);
            g_score = zeros(n,1);
            
            open_set(start) = 0;
            while find(isfinite(open_set))
                [M, I] = min(open_set(:));
                [i, j] = ind2sub(size(open_set), I);
                open_set(I) = Inf;
                closed_set(I) = 1;
                if I == goal
                    parent
                    break;
                end
                for k = 1:n
                    if k ~= I
                        cost = g_score(I)+graph(I,k);
                        if g_score(k) == 0 || cost < g_score(k)
                            g_score(k) = cost;
                            f_score = cost;
                            open_set(k) = f_score;
                            parent(k) = I;
                        end
                    end
                end
            end
        end
        function feasible = isFeasibleTerrain(obj, terrain, throughUnknownTerrain)
            feasible = 1;
            %if terrain == 0 || terrain == 1
            if throughUnknownTerrain == 0
                if terrain == 1 || terrain == 2
                    feasible = 0;
                end
            else
                if terrain == 1
                    feasible = 0;
                end
            end
        end
        function cost = heuristic(obj, start, goal)
            %dx = abs(start(1) - goal(1));
            %dy = abs(start(2) - goal(2));
            %cost = 1 *(dx + dy) + (sqrt(2) - 2 * 1) * min(dx, dy);
            d = abs(start - goal);
            cost = 1 *(sum(d)) + (sqrt(2) - 2 * 1) * min(d);
        end
        function collision = checkCollision(obj, map, pose, buffer,unknown)
            collision = 0;
            for i = 0:360
                r = ((2*pi)/360)*i;
                x = floor(pose(1)+cos(r)*(buffer));
                y = floor(pose(2)+sin(r)*(buffer));
                %x = floor(pose(1)+cos(r)*(obj.robot.robot_size + buffer))+1;
                %y = floor(pose(2)+sin(r)*(obj.robot.robot_size + buffer))+1;
                if unknown == 0
                    if(map(x, y) == 1 || map(x, y) == 2)
                        collision = 1;
                    end
                else
                    if(map(x, y) == 1)
                        collision = 1;
                    end
                end
                %obj.robot.map.visibility_map(x, y) = 4;
                %draw(obj.robot, obj.robot.map);
                %pause(0.001);
            end
        end
        function collision = checkSquareCollision(obj, map, pose, buffer,unknown)
            [m, n] = size(map);
            collision = 0;
            range = buffer/2;
            for i = -range:range
                for j = -range:range
                    if pose(1) == 0
                        fprintf('null');
                    elseif pose(2) == 0
                        fprintf('2null');
                    end
                    n_i = floor(pose(1)+i)
                    n_j = floor(pose(2)+j)
                    if(pose(1)+i > 0 && pose(1)+i < m && pose(2)+j > 0 && pose(2)+j < n)
                        if unknown == 0
                            if(map(n_i, n_j) == 1 || map(n_i, n_j) == 2)
                                collision = 1;
                            end
                        else
                            if(map(n_i, n_j) == 1 || map(n_i, n_j) == 3)
                                collision = 1;
                            end
                        end
                    end
                end
            end
        end
        function point = getCollisionFreePoint(obj, map,  pose, range, buffer, unknown)
            collision = 1;
            for y = -range:range
                for x = -range:range
                    point_i = pose(1)+y;
                    point_j = pose(2)+x;
                    collision = obj.checkSquareCollision(map, [point_i point_j],buffer, unknown);
                    
                    if collision == 0
                        point = [point_i point_j] ;
                        return;
                    else
                        point = pose;
                    end
                end
            end
        end
        function path = extractPath(obj, start, goal, path_map)
            start = start+1;
            goal = goal+1;
            position = goal;
            path = [];
            [~,n] = size(path_map);
            while(~isequal(start, position))
                position
                index = path_map(position(1), position(2));
                if index < 1
                    path = [];
                    break;
                end
                i = floor(double(index) / double(n))+1;
                j = mod(index, n)+1;
                path = [path; double(j) double(i)];
                position = [j i];
            end
        end
    end
end

