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
        function path = plan(obj, start, goal, map)
            path = [];
            [m,n] = size(map);
            parent = ones(m,n);
            open_set = Inf(m,n);
            closed_set = Inf(m,n);;
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
                                if(obj.isFeasibleTerrain(map(n_i, n_j)) && ~obj.checkCollision(map, [n_i n_j],1))
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

            position = goal
            %%while(position(1) ~= start(1) && position(2) ~= start(2))
            while(~isequal(start, position))
                index = parent(position(1), position(2));
                i = floor(index / m);
                j = mod(index, m);
                path = [path; i j];
                position = [i j];
            end
        end
        function feasible = isFeasibleTerrain(obj, terrain)
            feasible = 1;
            %if terrain == 0 || terrain == 1
            if terrain == 1 || terrain == 2
                feasible = 0;
            end
        end
        function cost = heuristic(obj, start, goal)
            dx = abs(start(1) - goal(1));
            dy = abs(start(2) - goal(2));
            cost = 1 *(dx + dy) + (sqrt(2) - 2 * 1) * min(dx, dy);
        end
        function collision = checkCollision(obj, map, pose, buffer)
            collision = 0;
            for i = 0:360
                r = ((2*pi)/360)*i;
                x = int32(pose(1)+sin(r)*(obj.robot.robot_size + buffer))+1;
                y = int32(pose(2)+cos(r)*(obj.robot.robot_size + buffer))+1;
                if(map(x, y) == 1 || map(x, y) == 2)
                    collision = 1;
                end
            end
        end
    end
    
end

