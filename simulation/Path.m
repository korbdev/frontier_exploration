classdef Path < handle
   properties
       waypoints;
       costs;
   end
   methods
       function obj = Path(points)
           if nargin > 0
               obj.waypoints = points;
               obj.costs = size(points,1);
           end
       end
   end
end