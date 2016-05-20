classdef Robot < handle
    properties
        robot_size;
        pose;
        goal;
        frontier_point;
        map;
        %sensor_range;
        sensor
        trajectory;
        collision_radius;
    end
    methods
        function obj = Robot(robot_size, initial_pose, sensor, map)
            obj.robot_size = robot_size;
            obj.pose = initial_pose;
            obj.map = map;
            obj.sensor = sensor;
            obj.goal = [];
            obj.frontier_point = [];
            obj.collision_radius = 1;
        end
        
        function explore(obj)
            planner = Planner(obj);
            obj.sense(); %first sense
            explorer = Explorer(obj);
            frontier = explorer.getFrontier()
            draw(obj, obj.map);
            
            %{
            while frontier ~= 0
                goal_is_found = 0;
                while(~goal_is_found)
                    draw(obj, obj.map);
                    pause(0.01);
                    for k = 0:360
                        r = ((2*pi)/360)*k;
                        y = floor(frontier(2)+sin(r)*(2*obj.robot_size+obj.collision_radius));
                        x = floor(frontier(1)+cos(r)*(2*obj.robot_size+obj.collision_radius));
                        %obj.map.visibility_map(x, y) = 5;
                        %draw(obj, obj.map);
                        %pause(0.001);
                        collision = planner.checkCollision(obj.map.visibility_map, [x y], 1);
                        if ~collision
                            obj.goal = [x, y];
                            goal_is_found = 1;
                            break;
                        end
                    end
                    obj.collision_radius = obj.collision_radius+1;
                end
                path = planner.plan(obj.pose, obj.goal, obj.map.visibility_map);
                length = size(path,1);
                for i = length:-1:1
                    point = path(i,:);
                    
                    obj.map.visibility_map(point(1), point(2)) = 4;
                    obj.pose = [point(1), point(2)];
                    obj.sense();
                    draw(obj, obj.map);
                    pause(0.01);
                end
                
                %{
                path = r.moveToFrontier();
                length = size(path,1);
                draw(r, map);
                for i = length:-1:1
                    point = path(i,:);
                    
                    map.visibility_map(point(1), point(2)) = 4;
                    r.pose = [point(1), point(2)];
                    r.sense();
                    draw(r, map);
                    pause(0.01);
                end
                draw(r, map);
                %}
                frontier = explorer.getFrontier();
                obj.sense();
                pause(0.01);
            
            end
            %}
        end
        
        function path = moveToFrontier(obj)
            path = [];
            planner = Planner(obj);
            
            obj.sense();
            
            idx = find(obj.map.frontier_map==3);
            n = size(idx);
            %r = a + (b-a).*rand(N,1)
            random = floor(1 + (n-1).*rand(1));
            frontier = idx(random(1));
            
            [i,j] = ind2sub(size(obj.map.frontier_map), frontier);
            obj.goal = [i j];
            obj.frontier_point = [i j];
            draw(obj, obj.map);
            
            %collision = planner.checkCollision(map, pose, obj.robot_size+1);
            for k = 0:360
                r = ((2*pi)/360)*k;
                x = floor(i+sin(r)*(2*obj.robot_size+obj.collision_radius));
                y = floor(j+cos(r)*(2*obj.robot_size+obj.collision_radius));
                
                collision = planner.checkCollision(obj.map.visibility_map, [x y], 1);
                if ~collision
                    obj.goal = [x, y];
                    break;
                end
                if k == 360
                    k = 0;
                    obj.collision_radius = obj.collision_radius+1;
                end
            end
            draw(obj, obj.map);
            path = planner.plan(obj.pose, obj.goal, obj.map.visibility_map);
            
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
            obj.map.visibility_map = obj.sensor.update(obj.pose, obj.map.visibility_map);
            %obj.map.update(obj.pose, obj.sensor_range);
            obj.map.createFrontierMap(obj.pose);
        end
    end
end