classdef Explorer < handle
    properties
        robot;
        clusters;
        omega;
        theta;
        t_h;
    end
    methods
        function obj = Explorer(robot, omega, theta)
            obj.robot = robot;
            obj.clusters = [];

            obj.omega = omega;
            obj.theta = theta;
        end
        function frontier =  getFrontier(obj)
            global log;
            planner = Planner(obj.robot);
            obj.clusters = [];
            smooth_frontier = obj.robot.map.frontier_map.angle_histogram_gauss;
            
            i = 1;
            n = obj.robot.map.frontier_map.bins+1;
            while i < n && ~isempty(isnan(smooth_frontier))
                if smooth_frontier(i) ~= 0
                    cluster = Cluster();
                    while i < n && smooth_frontier(i) ~= 0
                        cluster.data = [cluster.data; smooth_frontier(i)];
                        cluster.range = [cluster.range; i];
                        i = i + 1;
                    end
                    obj.clusters = [obj.clusters; cluster];
                end
                i = i + 1;
            end
            
            num_cluster = size(obj.clusters,1);
            last = obj.clusters(num_cluster);
            if last.range(size(last.range,1)) == obj.robot.map.frontier_map.bins
                first = obj.clusters(1);
                first.data = [last.data; first.data];
                first.range = [last.range; first.range];
                obj.clusters = obj.clusters(1:num_cluster-1,:);
            end
            
            distances = [];
            cardinality = [];
            
            for i = 1:size(obj.clusters)
                cluster_i = obj.clusters(i);
                for j = 1:size(cluster_i.range)
                    val = cluster_i.range(j);
                    [y, x] = find(obj.robot.map.frontier_map.bin_map == val);
                    cluster_i.points_i = [cluster_i.points_i; y];
                    cluster_i.points_j = [cluster_i.points_j; x];
                end
                [k, l] = cluster_i.getCentroid();
                
                cluster_i.cluster_goal(1) = floor(k), cluster_i.cluster_goal(2) = floor(l);
                %{
               if obj.robot.map.visibility_map(cluster_i.cluster_goal(1),cluster_i.cluster_goal(2)) == 2
                   euclidian_distance = [];
                   for c = 1:size(cluster_i.points_i, 1)
                       euclidian_distance = [euclidian_distance norm([cluster_i.points_i(c) cluster_i.points_j(c)] - obj.robot.pose)];
                   end
                   [M, distance_idx] = min(euclidian_distance);
                   cluster_i.cluster_goal(1) = cluster_i.points_i(distance_idx);
                   cluster_i.cluster_goal(2) = cluster_i.points_j(distance_idx);
                   %obj.robot.map.visibility_map(floor(k), floor(l)) = 4;
                   cl_goal = cluster_i.cluster_goal
                   path = planner.plan(obj.robot.pose, cluster_i.cluster_goal, obj.robot.map.visibility_map, 1, 0);
               else
                   %obj.robot.map.visibility_map(floor(k), floor(l)) = 6;
                   cl_goal = cluster_i.cluster_goal
                   path = planner.plan(obj.robot.pose, cluster_i.cluster_goal, obj.robot.map.visibility_map, 1, 0);
               end
                %}
                
                cl_goal = cluster_i.cluster_goal
                path = planner.plan(obj.robot.pose, cluster_i.cluster_goal, obj.robot.map.visibility_map, 1, 0, 1);
                cluster_i.distance = size(path,1);
                distances = [distances; cluster_i.distance];
                cardinality = [cardinality; cluster_i.getCardinality()];
            end

            distances_norm = distances/max(distances);
            cardinality_norm = cardinality/max(cardinality);
            
            d_score = [];
            c_score = [];
            
            for i = 1:size(obj.clusters,1)
                if(distances_norm(i) == 0)
                    d = 0
                else
                    %d = 1 - obj.omega * distances_norm(i);
                    d = obj.omega * (1/distances_norm(i));
                    
                end
                if(cardinality_norm(i) == 0)
                    c = 0;
                else
                    c = obj.theta * cardinality_norm(i)
                end
                
                d_score = [d_score; d];
                c_score = [c_score; c];
                fprintf(log, 'Cluster %d, d=%d(%d), c=%d(%d), s=%d\n', i, obj.clusters(i).distance, distances_norm(i,:), obj.clusters(i).getCardinality(), cardinality_norm(i,:), (d+c));
            end
            
            score =  d_score + c_score;
            
            [M, I] = max(score);
            
            if isempty(I)
                frontier = [];
                return; %no frontier left
            end
            
            frontier_goal = obj.clusters(I).cluster_goal;
            obj.robot.map.visibility_map(frontier_goal(1),frontier_goal(2))
            if obj.robot.map.visibility_map(obj.clusters(I).cluster_goal(1),obj.clusters(I).cluster_goal(2)) == 2
                euclidian_distance = [];
                for c = 1:size(obj.clusters(I).points_i, 1)
                    euclidian_distance = [euclidian_distance norm([obj.clusters(I).points_i(c) obj.clusters(I).points_j(c)] - obj.robot.pose)];
                end
                [M, distance_idx] = min(euclidian_distance);
                obj.clusters(I).cluster_goal(1) = obj.clusters(I).points_i(distance_idx);
                obj.clusters(I).cluster_goal(2) = obj.clusters(I).points_j(distance_idx);
                %obj.robot.map.visibility_map(floor(k), floor(l)) = 4;
                frontier_goal = obj.clusters(I).cluster_goal;
            end
            %obj.robot.map.visibility_map(cluster_i.cluster_goal(1), cluster_i.cluster_goal(2)) = 4;
            fprintf(log, 'Cluster Index %d, Goal: [%d,%d]\n', I, frontier_goal(1), frontier_goal(2));
            frontier = [frontier_goal(1) frontier_goal(2)];
            obj.robot.goal = frontier;
            obj.robot.frontier_point = frontier;
        end
    end
end