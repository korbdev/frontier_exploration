classdef Frontiermap < handle
    properties
        map;
        bins;
        bin_map;
        theta_map;
        theta_list;
        radius_map;
        radius_list;
        frontier_map;
        angle_histogram;
        angle_histogram_gauss;
        angle_histogram_normalized_filtered;
    end
    methods
        function obj = Frontiermap(m, n, map, bins)
            obj.map = map;
            obj.bins = bins;
            obj.bin_map = Inf(m,n);
            obj.theta_map = Inf(m,n);
            obj.radius_map = Inf(m,n);
            obj.frontier_map = zeros(m,n);
            obj.angle_histogram = zeros(bins,1);
            obj.angle_histogram_normalized_filtered = zeros(bins,1);
            obj.theta_list = [];
            obj.radius_list = [];
        end
        function createFrontierMap(obj, pose, sigma, t_h)
            [m, n] = size(obj.map.visibility_map);
            obj.bin_map = Inf(m,n);
            obj.theta_map = Inf(m,n);
            obj.radius_map = Inf(m,n);
            obj.frontier_map = zeros(m,n);
            obj.angle_histogram = zeros(obj.bins,1);
            obj.theta_list = [];
            obj.radius_list = [];
            for i = 2:m-1
                for j = 2:n-1
                    if isFrontier(obj, i , j) == 1
                        
                        p = transformCoordinatesToRobot([i j], pose);
                        [theta, radius] = transformCartesianToPolar(p);
                        
                        deg = floor((theta/(2*pi))*360)+1;
                        
                        obj.incrementHistogramAt(deg);
                        obj.setFrontier(i,j,deg, theta, radius);
                    end
                end
            end
            
            %normalized_hist = normalize(obj.angle_histogram);
            
            normalized_hist = obj.angle_histogram/(max(obj.angle_histogram));
            
            t_h_idx = find(normalized_hist < t_h);
            obj.angle_histogram_normalized_filtered = normalized_hist;
            obj.angle_histogram_normalized_filtered(t_h_idx) = 0;
            
            obj.angle_histogram_gauss = gauss(obj.angle_histogram_normalized_filtered, 0, sigma);
            
            theta_polarplot = zeros(obj.bins, 1);
            for i = 1:obj.bins
                theta_polarplot(i) = (i/360)*(2*pi);
            end
            
            subplot(2,3,5), polar(theta_polarplot, obj.angle_histogram_gauss), view([0 -90]);
            subplot(2,3,6), polar(theta_polarplot, obj.angle_histogram_normalized_filtered), view([0 -90]);
        end
        function setFrontier(obj, i, j, bin, theta, radius)
            obj.bin_map(i, j) = bin;
            obj.theta_map(i, j) = theta;
            obj.radius_map(i, j) = radius;
            obj.frontier_map(i, j) = 3;
            obj.theta_list = [obj.theta_list theta];
            obj.radius_list = [obj.radius_list radius];
        end
        function [frontier, bin, theta, radius] = getFrontier(obj, i, j)
            frontier = obj.frontier_map(i, j);
            bin = obj.bin_map(i, j);
            theta = obj.theta_map(i, j);
            radius = obj.radius_map(i, j);
        end
        function incrementHistogramAt(obj, i)
            obj.angle_histogram(i) = obj.angle_histogram(i) + 1;
        end
        function f = isFrontier(obj, x, y)
            f = 0;
            if obj.map.visibility_map(x, y) == 3
                if obj.map.visibility_map(x-1, y) == 2
                    f = 1;
                end
                if obj.map.visibility_map(x+1, y) == 2
                    f = 1;
                end
                if obj.map.visibility_map(x, y+1) == 2
                    f = 1;
                end
                if obj.map.visibility_map(x, y-1) == 2
                    f = 1;
                end
            end
        end
        function draw(obj, h)
            global color_map;
            frontier_img = ind2rgb(obj.frontier_map, color_map);
            
            figure(h);
            imshow(frontier_img);
        end
    end
end