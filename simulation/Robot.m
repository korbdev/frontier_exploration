classdef Robot < handle
    properties
        robot_size;
        pose;
        map;
        sensor_range;
        color_map;
    end
    methods
        function obj = Robot(robot_size, initial_pose, map, sensor_range, color_map)
            obj.robot_size = robot_size;
            obj.pose = initial_pose;
            obj.map = map;
            obj.sensor_range = sensor_range;
            obj.color_map = color_map;
        end
        function pose = moveTo(obj, pose)
            obj.pose
        end
        function pose = moveY(obj, step_to_go)
            %SENSE
            obj.map.update(obj.pose+step_to_go, obj.sensor_range);
            %CHECK
            if(obj.checkCollision(1) == 0)
                %GO
                obj.pose = [obj.pose(1) obj.pose(2)+step_to_go];
            end
            %{
           rgb_img = ind2rgb(obj.map.visibility_map, obj.color_map);
           shapeInserter = vision.ShapeInserter('Shape','Circles');
           circle = int32([obj.pose(2) obj.pose(1) obj.robot_size]);
           new_image = step(shapeInserter, rgb_img, circle);
           
           %hold on
           figure(2);
           imshow(new_image);
            %}
            pose = obj.pose;
        end
        function collision = checkCollision(obj, buffer)
            collision = 0;
            for i = 0:360
                r = ((2*pi)/360)*i;
                x = int32(obj.pose(1)+sin(r)*obj.robot_size + buffer);
                y = int32(obj.pose(2)+cos(r)*obj.robot_size + buffer);
                if(obj.map.visibility_map(x, y) == 1)
                    collision = 1;
                end
            end
        end
    end
end