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
        function map = generateSensingMap(obj, pose, radius)
            global color_map;
            [m, n] = size(obj.visibility_map);
            map = zeros(m,n);
            for i = 1:m
                for j = 1:n
                    dist = JFrontier.getDistance(j, i, pose(2), pose(1));
                    if dist <= radius
                        map(i, j) = obj.visibility_map(i, j);
                    else
                        map(i, j) = 2;
                    end
                end
            end
        end
    end
end