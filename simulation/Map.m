classdef Map < handle
    properties
        occupancy_map;
        visibility_map;
        frontier_map;
        frontier_polar;
        frontier_polar_gauss;
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
        
        function createFrontierMap(obj, pose)
            bins = 360;
            angle_hist = zeros(bins,1);
            obj.frontier_polar = [];
            obj.frontier_map = zeros(size(obj.occupancy_map));
            
            [n, m] = size(obj.occupancy_map);
            for i = 2:m-1
                for j = 2:n-1
                     if isFrontier(obj, i , j) == 1
                       obj.frontier_map(i, j) = 2;

                       p = transformCoordinatesToRobot(obj, [i j], pose);
                       [theta, radius] = transformCartesianToPolar(obj, p);
                       obj.frontier_polar = [obj.frontier_polar; theta];
                       
                       deg = floor((theta/(2*pi))*360)+1;
                       angle_hist(deg) = angle_hist(deg) + 1; %deg +1 => 1 based index
                     end
                end
            end

            sigma = 6.0;
            obj.frontier_polar_gauss = gauss(angle_hist, 0, sigma);

            theta_polarplot = zeros(bins, 1);
            for i = 1:bins
               theta_polarplot(i) = (i/360)*(2*pi);
            end
            
            subplot(2,2,4), polar(theta_polarplot, obj.frontier_polar_gauss), view([90 90]);
        end
        
        function p = transformCoordinatesToRobot(obj, point_to_transform, parent)
            angle = 0;
            x = point_to_transform(1)*cos(angle)+point_to_transform(2)*sin(angle);
            y = -point_to_transform(1)*sin(angle)+point_to_transform(2)*cos(angle);
            p = [x, y]-parent;
        end
        
        function [theta, radius] = transformCartesianToPolar(obj, p)
            radius = norm(p);
            theta = 0;
            if(p(2) >= 0)
                theta = acos(p(1)/radius);
            else
                theta = 2*pi - acos(p(1)/radius);
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
                    if(distance < radius)
                        if(obj.occupancy_map(x,y) == 1)
                            obj.visibility_map(x,y) = 1;
                            break;
                        else
                            obj.visibility_map(x,y) = 3;
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
                    if(distance < radius)
                        %break;
                        if(obj.occupancy_map(x,y) == 1)
                            obj.visibility_map(x,y) = 1;
                            break;
                        else
                            obj.visibility_map(x,y) = 3;
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
            %obj.visibility_map(pose(1), pose(2)) = 4;
        end
    end
end