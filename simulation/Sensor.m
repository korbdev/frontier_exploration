classdef Sensor < handle
    properties
        occupancy_map;
        num_free_pixels;
        radius;
        angle;
        wall;
        unknown;
        free;
    end
    methods
        function obj = Sensor(rgbimage, radius)
            obj.occupancy_map = rgbimage+1;
            obj.radius = radius;
            obj.angle
            obj.wall = 1;
            obj.unknown = 2;
            obj.free = 3;
            obj.num_free_pixels = sum(sum(obj.occupancy_map == 3));
        end
        
        function map =  update(obj, pose, map)
            [m, n] = size(obj.occupancy_map);
            
            for j = 1:m
                i = n;
                map = updateView(obj, pose, [j i], map);
            end
            
            for j = 1:m
                i = 1;
                map = updateView(obj, pose, [j i], map);
            end
            
            for i = 1:n
                j = m;
                map = updateView(obj, pose, [j i], map);
            end
            
            for i = 1:n
                j = 1;
                map = updateView(obj, pose, [j i], map);
            end
            
            free_territory = 0;
            occupied_territory = 0;
            n_wall = 0;
            n_free = 0;
            n_wall_wall = 0;
            n_wall_free = 0;
            n_free_wall = 0;
            n_free_free = 0;
            sum = 0;
            for i = 2:m-1
                for j = 2:n-1
                    f = 0;
                    if map(i, j) == obj.wall || map(i, j) == obj.free
                        if map(i-1, j) == obj.wall
                            f = 1;
                        end
                        if map(i+1, j) == obj.wall
                            f = 1;
                        end
                        if map(i, j+1) == obj.wall
                            f = 1;
                        end
                        if map(i, j-1) == obj.wall
                            f = 1;
                        end
                        if f==1
                            n_wall = n_wall +1;
                        else
                            n_free = n_free +1;
                        end
                        sum = sum+1;
                    end
                    if map(i, j) == obj.wall
                        %sum = sum +1;
                        occupied_territory = occupied_territory +1;
                        if f == 1
                            n_wall_wall = n_wall_wall +1;
                        else
                            n_wall_free = n_wall_free +1;
                        end
                    elseif map(i, j) == obj.free
                        if f == 1
                            n_free_wall = n_free_wall +1;
                        else
                            n_free_free = n_free_free +1;
                        end
                        %sum = sum+1;
                        free_territory = free_territory+1;
                    end
                end
            end
            
            %free_territory, occupied_territory, n_free, n_wall, n_wall_free, n_wall_wall, n_free_free, n_free_free;
            p_free = free_territory/sum;
            p_n_free = n_free/sum;
            p_wall = occupied_territory/sum;
            p_n_wall = n_wall/sum;
            p_n_w_w = n_wall_wall/sum;
            p_n_f = n_free/sum;
            p_n_f_f = n_free_free/sum;
            p_n_f_w = n_free_wall/sum;
            
            p_f_n_f = (p_free*p_n_f_f)/p_n_f;
            
        end
        
        function map = updateView(obj, pose, pose_to, map)
            x = pose(1);
            y = pose(2);
            x_end = pose_to(1);
            y_end = pose_to(2);
            
            dx = x_end - x;
            dy = y_end - y;
            
            distance = sqrt(double(dx)^2+double(dy)^2);
            
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
                    distance = sqrt(double(new_dx)^2+double(new_dy)^2);
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
                    f = f + b;
                    if(f > 0)
                        y = y + y_step;
                        f = f - a;
                    end
                    x = x + x_step;
                    %end %
                end
            else
                f = -dy;
                while y ~= y_end
                    new_dx = x - pose(1);
                    new_dy = y - pose(2);
                    distance = sqrt(double(new_dx)^2+double(new_dy)^2);
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
                    f = f + a;
                    if(f > 0)
                        x = x + x_step;
                        f = f - b;
                    end
                    y = y + y_step;
                    %end %
                end
            end
        end
    end
end