classdef Robot < handle
    properties
        robot_size;
        pose;
        map;
        sensor_range;
        trajectory;
    end
    methods
        function obj = Robot(robot_size, initial_pose, map, sensor_range)
            obj.robot_size = robot_size;
            obj.pose = initial_pose;
            obj.map = map;
            obj.sensor_range = sensor_range;
        end
        
        function path = moveToFrontier(obj)
           path = [];
           planner = Planner(obj);
           
           obj.sense();
           
           idx = find(obj.map.frontier_map==2);
           n = size(idx);
           %r = a + (b-a).*rand(N,1)
           random = floor(1 + (n-1).*rand(1));
           frontier = idx(random(1));
           
           [i,j] = ind2sub(size(obj.map.frontier_map), frontier);
           path = planner.plan(obj.pose, [i j], obj.map.visibility_map);
           
        end
        
        function path = moveTo(obj, goal)
            planner = Planner(obj);
            path = planner.plan(obj.pose, goal, obj.map.occupancy_map);
        end
        
        function pose = moveY(obj, step_to_go)
            %SENSE
            obj.map.update(obj.pose, obj.sensor_range);
            obj.map.createFrontierMap();
            %CHECK
            if(obj.checkCollision(1) == 0)
                %GO
                obj.pose = [obj.pose(1) obj.pose(2)+step_to_go];
                obj.trajectory = [obj.trajectory; obj.pose];
            end
            pose = obj.pose;
        end
        
        function collision = checkCollision(obj, buffer)
            collision = 0;
            for i = 0:360
                r = ((2*pi)/360)*i;
                x = int32(obj.pose(1)+sin(r)*obj.robot_size + buffer)+1;
                y = int32(obj.pose(2)+cos(r)*obj.robot_size + buffer)+1;
                if(obj.map.visibility_map(x, y) == 1)
                    collision = 1;
                end
            end
        end
        
        function sense(obj)
           obj.map.update(obj.pose, obj.sensor_range);
           obj.map.createFrontierMap(); 
        end
    end
end