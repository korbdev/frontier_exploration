classdef Explorer < handle
    properties
        robot;
    end
    methods
        function obj = Explorer(robot)
            obj.robot = robot;
        end
        function frontier =  getFrontier(obj)
            %frontier = obj.robot.pose;
            %obj.robot.goal = frontier;
            %obj.robot.frontier_point = frontier;
            
            smooth_frontier = obj.robot.map.frontier_polar_gauss;
            %clusters = [];
            i = 1;
            n = 360;
            while i < n
                if smooth_frontier(i) ~= 0
                    cluster = [];
                    range = [];
                    while smooth_frontier(i) ~= 0 && i < n
                       cluster = [cluster smooth_frontier(i)]; 
                       range = [range i];
                       i = i + 1;
                    end
                    centroid = mean(cluster)
                    avg_angle = mean(range)
                    weighted_centroid = mean(range .* cluster)
                end
                i = i + 1;
            end
            frontier = obj.robot.pose;
            obj.robot.goal = frontier;
            obj.robot.frontier_point = frontier;
            
            
            %{
            idx = find(obj.robot.map.frontier_map==3);
            n = size(idx);
            %r = a + (b-a).*rand(N,1)
            random = floor(1 + (n-1).*rand(1));
            random_idx = random(1)
            if random_idx == 0  
                frontier = [0 0];
                return;
            end
            frontier = idx(random_idx);
            
            [i,j] = ind2sub(size(obj.robot.map.frontier_map), frontier);
            frontier = [i j];
            obj.robot.goal = [i j];
            obj.robot.frontier_point = [i j];
            %}
        end

    end
end