classdef Robot < handle
    properties
        pose;
        map;
    end
    methods
        function obj = Robot(initial_pose, visibility_map)
          obj.pose = initial_pose;
          obj.map = visibility_map;
        end
        function pose = moveTo(obj, pose)
            obj.pose
        end
        function pose = moveY(obj, step)
           x = pose(1);
           y = pose(2);
           x_new = x+step;
           y_new = y+step;
           
           
           
        end
    end
end