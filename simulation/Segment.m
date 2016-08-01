classdef Segment < handle
    properties
        id;
        points;
        label;
        min;
        max;
        ratio;
        max_area;
        current_area;
    end
    methods
        function obj = Segment(id)
            if nargin > 0
                obj.id = id;
            else
                obj.id = [];
            end
            obj.points = [];
            obj.min = [Inf Inf];
            obj.max = [-Inf -Inf];
            obj.max_area = 0;
            obj.current_area = 0;
        end
        function addPoint(obj, i, j)
            obj.points = [obj.points; i j];
            if i < obj.min(1)
                obj.min(1) = i;
            end
            if i > obj.max(1)
                obj.max(1) = i;
            end
            if j < obj.min(2)
                obj.min(2) = j;
            end
            if j > obj.max(2)
                obj.max(2) = j;
            end
            sides = (obj.max - obj.min)+1;
            
            obj.ratio = sides(1)/sides(2);
            obj.max_area = sides(1)*sides(2);
            obj.current_area = size(obj.points, 1);
            
            if obj.ratio > 0.5 && obj.ratio < 2
                if obj.current_area/obj.max_area <= 0.5
                    obj.label = 'corridor';
                else
                    obj.label = 'room';
                end
            else
                obj.label = 'corridor';
            end
        end
    end
end