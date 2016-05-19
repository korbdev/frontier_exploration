classdef Explorer < handle
    properties
        robot;
        frontier_map;
        a_hist;
        a_hist_norm;
        a_hist_gauss;
    end
    methods
        function obj = Explorer(robot, map)
            obj.robot = robot;
            obj.frontier_map = map;
        end
        function explore()
            
        end
    end
end