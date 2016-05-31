classdef Cluster < handle
   properties
      data;
      range;
      points_i;
      points_j;
      distance;
      centroid;
      cluster_goal;
   end
   methods
       function obj = Cluster()
           obj.data = [];
           obj.range = [];
           obj.points_i = [];
           obj.points_j = [];
           obj.centroid = Inf;
           obj.distance = Inf;
           obj.centroid = [0 0];
           obj.cluster_goal = [];
       end
       function [i, j] = getCentroid(obj)
           if obj.centroid == 0
               i = mean(obj.points_i);
               j = mean(obj.points_j);
           else
               i = obj.centroid(1);
               j = obj.centroid(2);
           end
           
       end
       function k = getCardinality(obj)
           k = size(obj.points_i,1); 
       end
   end
end