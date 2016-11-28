classdef Robot < handle
    properties
        robot_size;
        pose;
        orientation;
        goal;
        frontier_point;
        map;
        sensor
        trajectory;
        collision_radius;
        
        complete_path;
        current_path;
        
        shorter_path;
        path_calc_counter;
    end
    methods
        function obj = Robot(robot_size, initial_pose, sensor, map)
            [m, n] = size(map.visibility_map);
            obj.robot_size = robot_size;
            obj.pose = initial_pose;
            %obj.orientation = -3.9270;
            obj.orientation = 0;
            obj.map = map;
            obj.sensor = sensor;
            obj.goal = [0 0];
            obj.frontier_point = [0 0];
            obj.collision_radius = 1;
            obj.complete_path = zeros(m, n);
            obj.current_path = zeros(m, n);
            obj.shorter_path = 0;
            obj.path_calc_counter = 0;
        end
        function obj = move(obj, old_pose, new_pose)
            dir = new_pose - old_pose;
            [theta, rho] = cart2pol(dir(2), dir(1));
            obj.pose = new_pose;
            if theta > pi && theta >= 0
                obj.orientation = theta - 2*pi;
            elseif theta < -pi && theta <= 0
                obj.orientation = theta + 2*pi;
            else
                obj.orientation = theta;
            end
        end
        function exploreCloseFrontiers3D(obj)
            global log;
            global color_map;
            global total_path_length;
            global calc_counter;
            %planner = Planner(obj);
            
            avg_vec = [];
            step_vec = []
            journey_vec = [];
            
            clims = [0 500];
            calc_counter = 0;
            percent = 0;
            Y = [];
            Y_percent = [];
            Y_avg_travel = [];
            Y_travel = [];
            Y_total_pixel = [];

            obj.sense(); %first sense
            draw(obj, obj.map);
            
            obj.orientation = pi/2;
            obj.sense();
            
            obj.orientation = pi;
            obj.sense();
            
            obj.orientation = -pi/2;
            obj.sense();
 
            draw(obj, obj.map);
            
            [m, n] = size(obj.map.visibility_map);
            planner = PathPlanner(obj.map.visibility_map, m, n, obj.robot_size+1, true);
            r_map = planner.planCostMap(obj.pose(1), obj.pose(2), false);
            
            obj.map.frontier_map.createFrontierMap(obj.pose, 0, 0, planner.safety_map);
            num_frontiers = size(obj.map.frontier_map.frontiers,1);
            
            journey_sum = 0;
            
            old_segment = 0;
            current_segment = 0;
            old_pose = obj.pose;
            while num_frontiers > 0
                calc_counter = calc_counter +1;

                subplot(2, 4, 4);
                imagesc(r_map);

                frontiers = obj.map.frontier_map.frontiers;
                dist_robot_frontier = Inf;
                min_dist_frontier_idx = 0;
                paths_length = [];
                for i=1:size(frontiers,1)
                    temp_frontier = frontiers(i);

                    distance = planner.reachability_map(temp_frontier.center(1), temp_frontier.center(2));
                    paths_length = [paths_length; distance];

                    if distance < dist_robot_frontier
                        dist_robot_frontier = distance;
                        min_dist_frontier_idx = i;
                    end
                end
                
                frontier = obj.map.frontier_map.frontiers(min_dist_frontier_idx);
                
                dist_robot_frontier = planner.reachability_map(frontier.center(1), frontier.center(2));
                journey_sum = journey_sum + dist_robot_frontier;
                avg_journey = journey_sum/calc_counter
                
                avg_vec = [avg_vec avg_journey];
                step_vec = [step_vec dist_robot_frontier]
                journey_vec = [journey_vec journey_sum];
                
                save('data_NN.mat', 'avg_vec', 'step_vec', 'journey_vec');
                
                subplot(2,4,5)
                X = 1:1:calc_counter;
                Y_percent = [Y_percent percent];
                Y = [Y journey_sum];
                plot(X, Y_percent, 'r', X, Y, 'b');
                
                subplot(2,4,6)
                X = 1:1:calc_counter;
                Y_avg_travel = [Y_avg_travel avg_journey];
                Y_travel = [Y_travel dist_robot_frontier];
                plot(X, Y_travel, 'r', X, Y_avg_travel, 'b');
                
                path = planner.generatePath(frontier.center(1), frontier.center(2));   
                
                if isequal(frontier.center, obj.pose)
                    %rotate
                    obj.orientation = 0;
                    obj.sense();
                    
                    obj.orientation = pi/2;
                    obj.sense();

                    obj.orientation = pi;
                    obj.sense();

                    obj.orientation = -pi/2;
                    obj.sense();
                end
                
                for i = size(path,1):-1:1
                    point = path(i,:);
                    total_path_length = total_path_length + 1;
                    dir = [double(point(1)), double(point(2))] - obj.pose;
                    [theta, rho] = cart2pol(dir(2), dir(1));
                    if theta > pi && theta >= 0
                        obj.orientation = theta - 2*pi;
                    elseif theta < -pi && theta <= 0
                        obj.orientation = theta + 2*pi;
                    else
                        obj.orientation = theta;
                    end
                    obj.pose = [double(point(1)), double(point(2))];
                    obj.complete_path(point(1), point(2)) = 3;
                    obj.sense();
                    %draw(obj, obj.map);
                    %pause(0.001);
                end
                   
                planner.map = obj.map.visibility_map;
                r_map = planner.planCostMap(obj.pose(1), obj.pose(2), false);
                sm = planner.safety_map;
                
                free_pixels = sum(sum(obj.map.visibility_map == 3));
                old_percent = percent;
                percent = free_pixels/obj.sensor.num_free_pixels;
                
                fprintf(log,'iteration %d, path travelled %d, percent explored %f(+%f)\n', calc_counter,total_path_length, percent, ((percent-old_percent)*100));
                
                obj.map.frontier_map.createFrontierMap(obj.pose, 0, 0, sm);
                
                num_frontiers = size(obj.map.frontier_map.frontiers,1);
                draw(obj, obj.map);
                pause(0.01);
            end
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
            obj.pose
            obj.orientation
            obj.map.visibility_map = obj.sensor.update(obj.pose, obj.orientation, obj.map.visibility_map);
        end
        
        function image = drawRobot(obj, h, image)
            global color_map;
            figure(h);
            circles = [];
            
            complete = find(obj.complete_path==3);
            current = find(obj.current_path==3);
            
            image(complete) = 6;
            image(current) = 4;
            
            image(obj.pose(1), obj.pose(2)) = 5;
            
            for i = 1:size(obj.map.frontier_map.frontiers, 1)
                f = obj.map.frontier_map.frontiers(i);
                image(floor(f.mean(1)), floor(f.mean(2))) = 5;
                image(f.center(1), f.center(2)) = 4;
                %circles = [circles;f.center(2), f.center(1) obj.sensor.radius;];
            end
            
            circles = [circles; obj.pose(2), obj.pose(1) obj.sensor.radius;];
            circles = [circles; obj.pose(2) obj.pose(1) obj.robot_size;];
            
            if obj.goal ~= 0
                %image(obj.goal(1), obj.goal(2)) = 4;
                circles = [circles; obj.goal(2) obj.goal(1) obj.robot_size;];
            end
            
            if obj.frontier_point ~= 0
                circles = [circles; obj.frontier_point(2) obj.frontier_point(1) obj.robot_size*2+obj.collision_radius;];
            end
            
            
            if ~isempty(circles)
                rgb_img = ind2rgb(image, color_map);
                shapeInserter = vision.ShapeInserter('Shape','Circles');
                new_image = step(shapeInserter, rgb_img, circles);
                plot([1 1], [240,240]);
                image = (rgb2ind(new_image, color_map));
            end
            
        end
    end
end