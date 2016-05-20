classdef Sensor < handle
    properties
        occupancy_map;
        radius;
        wall;
        unknown;
        free;
    end
    methods
        function obj = Sensor(rgbimage, radius)
            obj.occupancy_map = rgbimage+1;
            obj.radius = radius;
            obj.wall = 1;
            obj.unknown = 2;
            obj.free = 3;
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
        end
        
        function map = updateView(obj, pose, pose_to, map)
            x = pose(1);
            y = pose(2);
            x_end = pose_to(1);
            y_end = pose_to(2);
            
            dx = x_end - x;
            dy = y_end - y;
            
            distance = sqrt(dx^2+dy^2);
            
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