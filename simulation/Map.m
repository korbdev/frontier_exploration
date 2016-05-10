classdef Map < handle
    properties
        occupancy_map;
        visibility_map;
        frontier_map;
    end
    methods
        function obj = Map(indeximage)
            %Occupancy Map
            obj.occupancy_map = indeximage;
            %Explored Map
            obj.visibility_map = ones(size(obj.occupancy_map));
            obj.visibility_map = obj.visibility_map*2;
            %Frontier Map 
            obj.frontier_map = zeros(size(obj.occupancy_map));
        end
        
        function createFrontierMap(obj)
            obj.frontier_map = zeros(size(obj.occupancy_map));
            [n, m] = size(obj.occupancy_map);
            for j = 2:m-1
                for i = 2:n-1
                     if isFrontier(obj, j , i) == 1
                       obj.frontier_map(j, i) = 2;
                     end
                end
            end
        end
        
        function f = isFrontier(obj, x, y)
            f = 0;
            if obj.visibility_map(x, y) == 3
               if obj.visibility_map(x-1, y) == 2
                   f = 1; 
               end
               if obj.visibility_map(x+1, y) == 2
                   f = 1; 
               end
               if obj.visibility_map(x, y+1) == 2
                   f = 1; 
               end
               if obj.visibility_map(x, y-1) == 2
                   f = 1; 
               end
            end
        end
        
        function update(obj, pose, radius)
           [n, m] = size(obj.occupancy_map);
           
           for j = 1:m
              i = n;
              updateView(obj, pose, [j i], radius);
           end
           
           for j = 1:m
              i = 1;
              updateView(obj, pose, [j i], radius);
           end
           
           for i = 1:n
              j = m;
              updateView(obj, pose, [j i], radius);
           end
           
           for i = 1:n
              j = 1;
              updateView(obj, pose, [j i], radius);
           end
        end
        
        function updateView(obj, pose, pose_to, radius)
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
                while x ~= x_end
                    new_dx = x - pose(1);
                    new_dy = y - pose(2);
                    distance = sqrt(new_dx^2+new_dy^2);
                    if(distance < radius)
                        if(obj.occupancy_map(x,y) == 1)
                            obj.visibility_map(x,y) = 1;
                            break;
                        else
                            obj.visibility_map(x,y) = 3;
                        end
                    end
                    f = f + b;
                    if(f > 0)
                        y = y + y_step;
                        f = f - a;
                    end
                    x = x + x_step;
                end
            else
                f = -dy;
                while y ~= y_end
                    new_dx = x - pose(1);
                    new_dy = y - pose(2);
                    distance = sqrt(new_dx^2+new_dy^2);
                    if(distance < radius)
                        %break;
                        if(obj.occupancy_map(x,y) == 1)
                            obj.visibility_map(x,y) = 1;
                            break;
                        else
                            obj.visibility_map(x,y) = 3;
                        end
                    end
                    f = f + a;
                    if(f > 0)
                        x = x + x_step;
                        f = f - b;
                    end
                    y = y + y_step;
                end
            end
            %obj.visibility_map(pose(1), pose(2)) = 4;
        end
    end
end