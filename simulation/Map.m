classdef Map < handle
    properties
        visibility_map;
        frontier_map;
    end
    methods
        function obj = Map(m, n)
            %Explored Map
            obj.visibility_map = ones(m, n)*2; %initialize territory as unknown
            %Frontier Map 
            obj.frontier_map = Frontiermap(m, n, obj, 360);
        end
    end
end