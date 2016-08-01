classdef Frontier < handle
   properties
       id
       points
       outer_points
       inner_points
       endpoints
       mean
       outer_mean
       inner_mean
       center
       direction
       color
   end
   methods
       function obj = Frontier(id, points, color, map)
          if nargin > 1
              obj.id = id;
              obj.points = points;
              obj.outer_points = [];
              obj.inner_points = [];
              obj.endpoints = [];
              obj.color = color;
              obj.mean = mean(points,1);
              [~, idx] = obj.getClosestPoint(obj.mean);
              obj.center = obj.points(idx,:);
              obj.getOuterFrontier(map);
              obj.direction = obj.getFrontierDirection(map, 1);
          end
       end
       function [dist,idx] = getClosestPoint(obj, point)
          point_matrix = repmat(point, size(obj.points,1), 1);
          temp = sqrt(sum((obj.points - point_matrix).^2,2));
          [dist, idx] = min(temp);
       end
       function dist = getDistance(obj, point)
           dist = sqrt(sum((obj.center - point).^2,2));
       end
       function setEndpoints(obj, endpoints)
           obj.endpoints = endpoints;
       end
       function getOuterFrontier(obj, map)
           for p = 1:size(obj.points,1)
               point = obj.points(p,:);
               for i = -1:1
                   for j = -1:1
                       n_i = point(1) + i;
                       n_j = point(2) + j;
                       if i ~= 0 || j ~= 0
                           if map(n_i, n_j) == 3 || map(n_i, n_j) == 1
                               obj.inner_points = [obj.inner_points; n_i n_j];
                               %direction_counter = direction_counter + 1;
                               %direction = [n_i n_j] - obj.center
                               %direction_sum = direction_sum + direction;
                           end
                           if map(n_i, n_j) == 2 || map(n_i, n_j) == 1
                               obj.outer_points = [obj.outer_points; n_i n_j];
                           end
                       end
                   end
               end
           end
           obj.outer_points = unique(obj.outer_points, 'rows');
           obj.inner_points = unique(obj.inner_points, 'rows');
           obj.outer_mean = mean(obj.outer_points);
           obj.inner_mean = mean(obj.inner_points);
           
       end
       function frontier_direction = getFrontierDirection(obj, map, neighbourhood)
           %berechne mittelwert der frontierpunkte
           %berechne richtung von mittelwer und center -> richtung der frontier

           direction_sum = 0;
           direction_counter = 0;
           for i = -neighbourhood:neighbourhood
               for j = -neighbourhood:neighbourhood
                   n_i = obj.center(1) + i;
                   n_j = obj.center(2) + j;
                   if i ~= 0 || j ~= 0
                       if map(n_i, n_j) == 2
                           direction_counter = direction_counter + 1;
                           direction = [n_i n_j] - obj.center;
                           direction_sum = direction_sum + direction;
                       end
                   end
               end
           end
           frontier_direction = direction_sum./direction_counter;
           fdir = obj.outer_mean - obj.inner_mean;
           frontier_direction = fdir./sqrt(sum(fdir.^2));
       end
   end
end