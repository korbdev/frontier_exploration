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
        function pose = moveTo(obj, pose)
            obj.pose
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
    end
end