classdef Robot < handle
    properties
        robot_size;
        pose;
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
        function exploreCloseFrontiers(obj)
            global log;
            global color_map;
            global total_path_length;
            %planner = Planner(obj);
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
            
            [m, n] = size(obj.map.visibility_map);
            planner = PathPlanner(obj.map.visibility_map, m, n, obj.robot_size, true);
            r_map = planner.planCostMap(obj.pose(1), obj.pose(2), false);
            
            obj.map.frontier_map.createFrontierMap(obj.pose, 0, 0, planner.safety_map);
            num_frontiers = size(obj.map.frontier_map.frontiers,1);
            
            journey_sum = 0;
            
            old_segment = 0;
            current_segment = 0;
            old_pose = obj.pose;
            while num_frontiers > 0
                calc_counter = calc_counter +1;
               
                sm = planner.safety_map;
                subplot(2, 4, 4);
                imagesc(planner.safety_map);
                
                subplot(2, 4, 3);
                imagesc(r_map);
                
                grey_map = sm == 2;
                temp_sm = sm;
                temp_sm(grey_map) = 0;
                %perform segmentation
                current = ind2rgb(temp_sm, color_map);

                %BW = im2bw(current,0.5);

                %im = -bwdist(BW, 'euclidean');

                %mask = imextendedmin(im,2);
                
                %D2 = imimposemin(im ,mask);
                im = imgaussfilt(imcomplement(current), 4);
                wim = watershed(im,8);
                
                %wim = watershed(im, 8);
                
                
                %imshowpair(bw,mask,'blend')
                
                segmented = zeros(240);
                segments = Segment.empty;
                for i = 1:240
                    for j = 1:240
                        if obj.map.visibility_map(i, j) == 3
                            if wim(i, j) ~= 0
                                id = wim(i,j);

                                if id > size(segments, 2)
                                    seg = Segment(id);
                                    seg.addPoint(i, j);
                                    segments(id) = seg;
                                else
                                    seg = segments(id);
                                    seg.id = id;
                                    seg.addPoint(i, j);
                                end
                            end
                            segmented(i,j) = wim(i, j);
                        end
                    end
                end
                
                current_segment = segmented(obj.pose(1), obj.pose(2));
                old_segment = segmented(old_pose(1), old_pose(2));
                %corners = detectHarrisFeatures(BW);
                %[features, valid_corners] = extractFeatures(BW, corners);

                subplot(2,4,3);
                imagesc(im);
                %hold on;
                %plot(valid_corners);
                %hold off;

                subplot(2,4,4);
                bneck = planner.computeBottleNeckMap();
                %imagesc(bneck);
                imagesc(segmented);
                 % Make a truecolor all-green image.
                green = cat(3, zeros(size(bneck)), ones(size(bneck)), zeros(size(bneck)));
                hold on 
                h = imshow(green); 
                hold off
                 
                alphamap = bneck < 100;
                bneck(alphamap) = 0;
                set(h, 'AlphaData', bneck)
                %subplot(2,4,4);
                %imagesc(segmented);
                
                subplot(2,4,8);
                imagesc(im);
                
                %%frontier reachability maps
                %%For planning in unknown environment
                %frontier_rmaps = empty();
                
                %%calculate FrontierGraph
                FG = FrontierGraph(obj.map.frontier_map.frontiers, obj.map, planner);
                temp = (FG.weights > FG.weights_uspace);
                
                dp = FG.dot_product;
                
                frontiers = obj.map.frontier_map.frontiers;
                dist_robot_frontier = Inf;
                min_dist_frontier_idx = 0;
                paths_length = []
                for i=1:size(frontiers,1)
                    temp_frontier = frontiers(i);
                    %drawArrow = @(x,y) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0 ); 
                    %subplot(2, 4, 3);
                    %hold on;
                   % x1 = [temp_frontier.center(2) temp_frontier.center(2)+temp_frontier.direction(2)*10];
                    %y1 = [temp_frontier.center(1) temp_frontier.center(1)+temp_frontier.direction(1)*10];

                    %drawArrow(x1,y1);
                    distance = planner.reachability_map(temp_frontier.center(1), temp_frontier.center(2));
                    %hold off;
                    paths_length = [paths_length; distance];
                    %distance = size(temp_frontier.points,1)
                    if distance < dist_robot_frontier
                        dist_robot_frontier = distance;
                        min_dist_frontier_idx = i;
                    end
                end
                         
                %%find single frontiers
                single_frontier = any(temp,2);
                temp_paths_length = paths_length;
                temp_paths_length(single_frontier==1) = Inf;
                
                %if find(~any(single_frontier, 2))
                %    [~, safe_path_index] = min(temp_paths_length);
                %    idx = 1;
                %    val = Inf;
                %end
                
                
                %{
                journey_sum = journey_sum + dist_robot_frontier;
                avg_journey = journey_sum/calc_counter
                dist_robot_frontier
                
                subplot(2,4,3)
                X = 1:1:calc_counter;
                Y_percent = [Y_percent percent];
                Y = [Y journey_sum];
                plot(X, Y_percent, 'r', X, Y, 'b');
                
                subplot(2,4,4)
                X = 1:1:calc_counter;
                Y_avg_travel = [Y_avg_travel avg_journey];
                Y_travel = [Y_travel dist_robot_frontier];
                plot(X, Y_travel, 'r', X, Y_avg_travel, 'b');
                %}
                %if obj.sensor.radius-obj.robot_size < avg_journey
                    %journey_sum = 0;
                    %calc_counter = 0;
                    
                    dp_temp = dp <= 0;
                    test_vector = ones(1, size(FG.weights, 2));
                    sf_vector = ismember(dp_temp, test_vector, 'rows');
                    sf = find(any(dp_temp, 2))
                    if find(any(sf_vector, 2))
                        temp_paths_length = paths_length;
                        temp_paths_length(sf_vector==0) = Inf;
                        %[~, min_dist_frontier_idx] = min(temp_paths_length);
                        %idx = 1;
                        %val = Inf;
                    end
                %end
                
                frontier = obj.map.frontier_map.frontiers(min_dist_frontier_idx);
                
                %if old_segment ~= current_segment
                    %search = old_segment
                
                min_dist = Inf;
                for i=1:size(frontiers,1)
                    points = frontiers(i).points;
                    for j = size(points, 1)
                        p = points(j,:);
                        segment_robot = segmented(obj.pose(1), obj.pose(2));
                        segment_frontier = segmented(p(1), p(2));
                        if old_segment == segment_frontier
                            %frontier = frontiers(i);
                            dist = paths_length(i);
                            if dist < min_dist
                                frontier = frontiers(i);
                                min_dist = dist;
                            end
                        end
                    end
                end
                
                %{
                img_size = obj.sensor.radius*2;
                f_img = zeros(img_size);
                f_c = zeros(img_size);
                for i = -obj.sensor.radius:obj.sensor.radius
                    for j =-obj.sensor.radius:obj.sensor.radius
                        n_i = frontier.center(1)+i;
                        n_j = frontier.center(2)+j;
                        f_img_idx_i = i+obj.sensor.radius+1;
                        f_img_idx_j = j+obj.sensor.radius+1;
                        if n_i > 1 && n_i <= m && n_j > 1 && n_j <= n
                            if obj.map.visibility_map(n_i, n_j) == 1;
                               f_img(f_img_idx_i, f_img_idx_j) = 1;
                            else
                               f_img(f_img_idx_i, f_img_idx_j) = 0; 
                            end
                            f_c(f_img_idx_i, f_img_idx_j) = obj.map.visibility_map(n_i, n_j);
                        else
                            f_img(f_img_idx_i, f_img_idx_j) = 0;
                            f_c(f_img_idx_i, f_img_idx_j) = 2;
                        end
                    end
                end
                %}
                
                %{
                subplot(2, 4, 3);    
                [H,T,R] = hough(f_img);
                imshow(H,[],'XData',T,'YData',R,'InitialMagnification','fit');
                %xlabel('\theta'), ylabel('\rho');
                %axis on, axis normal, hold on;
                
                P  = houghpeaks(H,5,'threshold',ceil(0.7*max(H(:))));
                
                lines = houghlines(f_img,T,R,P,'FillGap',1,'MinLength',10);
                subplot(2, 4, 4);
                imshow(f_c), hold on
                max_len = 0;
                for k = 1:length(lines)
                   xy = [lines(k).point1; lines(k).point2];
                   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

                   % Plot beginnings and ends of lines
                   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
                   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

                   % Determine the endpoints of the longest line segment
                   len = norm(lines(k).point1 - lines(k).point2);
                   if ( len > max_len)
                      max_len = len;
                      xy_long = xy;
                   end
                end
                
                hold off;
                %subplot(2, 4, 4);
                %imagesc(f_img);
               % imagesc(f_img);
                %}
                
                dist_robot_frontier = planner.reachability_map(frontier.center(1), frontier.center(2));
                journey_sum = journey_sum + dist_robot_frontier;
                avg_journey = journey_sum/calc_counter
                
                path = planner.generatePath(frontier.center(1), frontier.center(2));   
                
                if min_dist == Inf || current_segment == old_segment
                    old_pose = obj.pose;
                end
                
                for i = size(path,1):-1:1
                    point = path(i,:);
                    total_path_length = total_path_length + 1;
                    obj.pose = [double(point(1)), double(point(2))];
                    obj.complete_path(point(1), point(2)) = 3;
                    obj.sense();
                    %draw(obj, obj.map);
                    %pause(0.001);
                end
                
                %old_segment = current_segment
                %current_segment = segmented(obj.pose(1), obj.pose(2));
                
                planner.map = obj.map.visibility_map;
                r_map = planner.planCostMap(obj.pose(1), obj.pose(2), false);
                sm = planner.safety_map;
                
                %subplot(2, 4, 4);
                %imagesc(planner.safety_map);
                
                %subplot(2, 4, 3);
                %imagesc(r_map);
                
                
                free_pixels = sum(sum(obj.map.visibility_map == 3));
                old_percent = percent;
                percent = free_pixels/obj.sensor.num_free_pixels;
                
                fprintf(log,'iteration %d, path travelled %d, percent explored %f(+%f)\n', calc_counter,total_path_length, percent, ((percent-old_percent)*100));
                
                %subplot(2,4,3)
                %X = 1:1:calc_counter;
                %Y_percent = [Y_percent percent];
                %Y = [Y journey_sum];
                %Y_total_pixel = [Y_total_pixel total_path_length];
                %plot(X, Y_total_pixel, 'r', X, Y, 'b');
                
                %subplot(2,4,4)
                %X = 1:1:calc_counter;
                %Y_avg_travel = [Y_avg_travel avg_journey];
                %Y_travel = [Y_travel dist_robot_frontier];
                %plot(X, Y_travel, 'r', X, Y_avg_travel, 'b');
                %ylim([0 500]);
                
                obj.map.frontier_map.createFrontierMap(obj.pose, 0, 0, sm);
                
                num_frontiers = size(obj.map.frontier_map.frontiers,1);
                draw(obj, obj.map);
                pause(0.01);
            end
        end
        function exploreBeeline(obj)
            global log;
            global color_map;
            global total_path_length;
            %planner = Planner(obj);
            clims = [0 500];
            calc_counter = 0;
            percent = 0;
            
            obj.sense(); %first sense
            draw(obj, obj.map);
            
            [m, n] = size(obj.map.visibility_map);
            planner = PathPlanner(obj.map.visibility_map, m, n, obj.robot_size, true);
            r_map = planner.planCostMap(obj.pose(1), obj.pose(2), false);
            
            obj.map.frontier_map.createFrontierMap(obj.pose, 0, 0, planner.safety_map);
            num_frontiers = size(obj.map.frontier_map.frontiers,1);
            while num_frontiers > 0
                calc_counter = calc_counter +1;
                free_pixels = sum(sum(obj.map.visibility_map == 3));
                old_percent = percent;
                percent = free_pixels/obj.sensor.num_free_pixels;
                
                fprintf(log,'iteration %d, path travelled %d, percent explored %f(+%f)\n', calc_counter,total_path_length, percent, ((percent-old_percent)*100));
               
                sm = planner.safety_map;
                subplot(2, 4, 4);
                imagesc(planner.safety_map);
                
                subplot(2, 4, 3);
                imagesc(r_map);
                
                frontiers = obj.map.frontier_map.frontiers;
                dist_robot_frontier = Inf;
                min_dist_frontier_idx = 0;
                for i=1:size(frontiers,1)
                    temp_frontier = frontiers(i);
                    distance = PathPlanner.getEuklidianDistance(temp_frontier.center(1), temp_frontier.center(2), obj.pose(1), obj.pose(2));
                    if distance < dist_robot_frontier
                        dist_robot_frontier = distance;
                        min_dist_frontier_idx = i;
                    end
                end
                                
                frontier = obj.map.frontier_map.frontiers(min_dist_frontier_idx);
                
                path = planner.generatePath(frontier.center(1), frontier.center(2));   
                planner_kspace = PathPlanner(obj.map.visibility_map, m, n, obj.robot_size, true);
                r_kmap = planner_kspace.planCostMap(frontier.center(1), frontier.center(2), false);
                %subplot(2, 4, 3);
                %imagesc(r_kmap, clims);
                %set(gca,'YTick',[]); set(gca,'XTick',[]);
                %colorbar;
                
                planner_uspace = PathPlanner(obj.map.visibility_map, m, n, obj.robot_size, true);
                r_umap = planner_uspace.planCostMap(frontier.center(1), frontier.center(2), true);
                %subplot(2, 4, 4);
                %imagesc(r_umap, clims);
                %set(gca,'YTick',[]); set(gca,'XTick',[]);
                %colorbar;
                %GO
                
                for i = size(path,1):-1:1
                    point = path(i,:);
                    total_path_length = total_path_length + 1;
                    obj.pose = [double(point(1)), double(point(2))];
                    obj.complete_path(point(1), point(2)) = 3;
                    obj.sense();
                end
                
                draw(obj, obj.map);
                planner.map = obj.map.visibility_map;
                r_map = planner.planCostMap(obj.pose(1), obj.pose(2), false);
                sm = planner.safety_map;
                
                subplot(2, 4, 4);
                imagesc(planner.safety_map);
                
                subplot(2, 4, 3);
                imagesc(r_map);
                
                obj.map.frontier_map.createFrontierMap(obj.pose, 0, 0, sm);
                num_frontiers = size(obj.map.frontier_map.frontiers,1);
                pause(0.01);
            end
        end
        function exploreUspace(obj)
            global log;
            global color_map;
            global total_path_length;
            planner = Planner(obj);
            
            calc_counter = 0;
            percent = 0;
            while 1
                calc_counter = calc_counter +1;
                
                obj.sense(); %first sense
                draw(obj, obj.map);
                free_pixels = sum(sum(obj.map.visibility_map == 3));
                old_percent = percent;
                percent = free_pixels/obj.sensor.num_free_pixels;
                
                fprintf(log,'iteration %d, path travelled %d, percent explored %f(+%f)\n', calc_counter,total_path_length, percent, ((percent-old_percent)*100));
                
                obj.map.frontier_map.createFrontierMap(obj.pose, 0, 0);
                FG = FrontierGraph(obj.map.frontier_map.frontiers, obj.map, planner);
                paths = Path.empty;
                paths_length = [];
                paths_uspace= Path.empty;
                path_length_uspace = [];
                
                %Calculate paths from current robot pose to frontiers
                for i = 1:size(obj.map.frontier_map.frontiers, 1)
                    f = obj.map.frontier_map.frontiers(i);
                    f_dist = f.getClosestPoint(obj.pose);
                    
                    
                    %find collission free frontier point
                    %{
                    for y = -6:6
                        for x = -6:6
                            n_i = f.center(1)+y;
                            n_j = f.center(2)+x;
                            collision = planner.checkSquareCollision(obj.map.visibility_map, [n_i n_j],2*2, 0);
                            if collision == 0
                               new_frontier = [n_i n_j]
                            end
                        end
                    end
                    %}
                    
                    new_frontier = planner.getCollisionFreePoint(obj.map.visibility_map, f.center, 2, 2*2, 0);
                    
                    %points_unknown = planner.plan(obj.pose, f.center, obj.map.visibility_map, 1, 0, 1);
                    %points_known = planner.plan(obj.pose, new_frontier, obj.map.visibility_map, 4, 1, 0);
                    
                    [r_map, path_map] = planner.planCostMap(obj.map.visibility_map, obj.pose, 0);
                    path = planner.extractPath(obj.pose, new_frontier, path_map);
                    paths = [paths;  Path(path)];
                    %paths = [paths;  Path(points_known)];
                    paths_length = [paths_length; size(path,1)];
                    %paths_length = [paths_length; size(points_known,1)];
                    
                    %paths_uspace = [paths_uspace; Path(points_unknown)];
                    %path_length_uspace = [path_length_uspace; size(points_unknown, 1)];
                    subplot(2, 4, 5);
                    imagesc(r_map);
                    colorbar;
                end
                
                %fprintf('calculate costmap');
                
                %cost_map = dijkstra(obj.map.visibility_map, 240, 240, obj.pose(2), obj.pose(1), 1);
                
                %subplot(2, 4, 6);
                %imagesc(cost_map);
                %colorbar;
                
                temp = (FG.weights > FG.weights_uspace);
                temp2 = (FG.weights < FG.weights_uspace);
                
                %{
                if ~any(temp(:))
                    [~, safe_path_index] = min(paths_length);
                    idx = 1;
                    val = Inf;
                else
                %}
                %{
                    diff = FG.weights - FG.weights_uspace;
                    [val, I] = max(diff(:));
                    [safe_path_index, idx] = ind2sub(size(diff), I);
                    if paths_length(idx) < paths_length(safe_path_index)
                        temp_idx = idx;
                        idx = safe_path_index;
                        safe_path_index = temp_idx;
                    end
                %}
                single_frontier = any(temp,2);
                temp_paths_length = paths_length;
                temp_paths_length(single_frontier==1) = Inf
                %if find(~any(single_frontier, 2))
                %    [~, safe_path_index] = min(temp_paths_length);
                %    idx = 1;
                %    val = Inf;
                %else
                %subplot(2, 4, 5)
                %x = linspace(0,sqrt(240^2+240^2));
                %a = exp(-0.02*x);
                %plot(x,a)
                
                p_exists = exp(-0.02*FG.weights_uspace);
                %p_temp = p_exists > 0.6;
                %p_exists(p_temp==0) = 0;
                
                %subplot(2, 4, 5)
                %[A,B] = meshgrid(0:10:240, 0:10:240);
                %Z = X .* exp(-X.^2 - Y.^2);
                %C = (A - B)./(A.*(40^2)-B)*0.7;
                %Z = -((Y./X)*0.7);
                %Z = (1-omega*X)+theta*Y;;
                %C = ((1-(B./A))*(0.7));
                %surf(A,B,C)
                %title('(B./A)*(1-0.7)');
                %xlabel('max-dist')
                %ylabel('min-dist')
                
                
                %subplot(2, 4, 6)
                %[X,Y] = meshgrid(0:10:240, 0:10:240);
                %Z = X .* exp(-X.^2 - Y.^2);
                %Z = (X - Y)./(X.*(40^2)-Y)*0.7;
                %Z = -((Y./X)*0.9);
                %Z = (1-omega*X)+theta*Y;;
                %surf(X,Y,Z)
                %title('(X - Y)./(X.*(40^2)-Y)*0.7');
                %xlabel('max-dist')
                %ylabel('min-dist')
                
                %cost = ((FG.weights - FG.min_dist)./(FG.weights*(40^2)-FG.min_dist)).*p_exists;
                %good ratio = (FG.min_dist./FG.weights);
                %good cost = ratio.*(1-p_exists);
                
                %ratio = (FG.min_dist./FG.weights);
                
                %ratio uspace/kspace
                %ratio = (FG.weights_uspace./FG.weights);
                
                %ratio optimal/uspace
                ratio = FG.min_dist./FG.min_dist_uspace;
                
                %cost function p uspace/kspace
                %cost = (1-ratio).*(p_exists);%.*(paths_length(safe_path_index));
                
                %cost optimal
                cost = ratio
                
                distance_penalty = 1./(repmat(paths_length, 1, size(paths_length, 1)));
                
                %cost_w_penalty = cost .* distance_penalty;
                cost_w_penalty = cost .* distance_penalty
                
                %cost = ((FG.weights_uspace - FG.min_dist)./(FG.weights-FG.min_dist)).*p_exists;
                cost(temp~=1) = NaN;
                cost_w_penalty(temp~=1) = NaN;
                temp_inf = cost_w_penalty == Inf;
                cost_w_penalty(temp_inf) = 0;
                %[val, I] = max(cost(:));
                [val, I] = max(cost_w_penalty(:));
                %[val, I] = max(cost_w_penalty(isfinite(cost_w_penalty(:))));
                [safe_path_index, idx] = ind2sub(size(cost), I);
                %if paths_length(idx) < paths_length(safe_path_index)
                %    temp_idx = idx;
                %    idx = safe_path_index;
                %    safe_path_index = temp_idx;
                %end
                
                %d_1 = obj.map.frontier_map.frontiers(safe_path_index).getDistance(obj.pose);
                %d_2 = obj.map.frontier_map.frontiers(idx).getDistance(obj.pose);
                %end
                
                %{
                val = Inf;
                safe_path_index = 1;
                idx = 1;
                while(val == Inf)
                    [~, safe_path_index] = min(paths_length);
                    paths_length(safe_path_index) = Inf;
                    temp = (FG.weights > FG.weights_uspace);
                    diff = FG.weights - FG.weights_uspace;
                    if ~any(temp(:))
                        break;
                    else
                        nbf = FG.weights_uspace;
                        nbf(temp==0) = Inf;
                        nbf = nbf(safe_path_index,:);
                        [val, idx] = min(nbf(:));
                    end
                end
                %}
                
                u_path = [safe_path_index, idx];
                u_path
                %drive the safe path to initial position
                
                safe_path = paths(safe_path_index);
                s_path_wp = safe_path.waypoints;
                for i = safe_path.costs:-1:1
                    point = s_path_wp(i,:);
                    total_path_length = total_path_length + 1;
                    obj.pose = [point(1), point(2)];
                    obj.complete_path(point(1), point(2)) = 3;
                    obj.sense();
                    draw(obj, obj.map);
                    pause(0.01);
                end
                
                p = FG.paths_uspace(safe_path_index,idx);
                path = p.waypoints;
                %check feasibility
                if path ~= 0
                    for i = 1:size(path, 1)
                        point = path(i,:);
                        if planner.isFeasibleTerrain(obj.map.visibility_map(point(1), point(2)), 1)==0
                            val = Inf;
                            break;
                        end
                    end
                end
                
                
                
                %path through uspace
                if ~isinf(val) && ~isnan(val)
                    if safe_path_index > idx
                        start = 1;
                        step = 1;
                        fin = p.costs
                    else
                        start = p.costs;
                        step = -1;
                        fin = 1
                    end
                    for i = start:step:fin
                        point = path(i,:);
                        %collision = planner.checkCollision(obj.map.visibility_map, point, 1, 0);
                        collision = planner.checkSquareCollision(obj.map.visibility_map, point,2*2, 0);
                        if collision == 0
                            total_path_length = total_path_length + 1;
                            obj.pose = [point(1), point(2)];
                            obj.complete_path(point(1), point(2)) = 3;
                        else
                            break;
                        end
                        obj.sense();
                        draw(obj, obj.map);
                        pause(0.01);
                    end
                end
                
                
                %{
                obj.sense(); %first sense
                obj.map.frontier_map.createFrontierMap(obj.pose, 0, 0);
                FG = FrontierGraph(obj.map.frontier_map.frontiers, obj.map, planner);
                paths = Path.empty;
                pathsUspace= Path.empty;
                
                shortestPath = Inf;
                pathIndex = 1;
                
                closestFrontier = Inf;
                frontierIndex = 1;
                
                %temp = (FG.weights > FG.weights_uspace);
                %temp2 = FG.weights_uspace;
                %temp2(temp==0) = Inf;
                %[ig, idx] = min(temp2(:));
                for i = 1:size(obj.map.frontier_map.frontiers, 1)
                    f = obj.map.frontier_map.frontiers(i);
                    f_dist = f.getClosestPoint(obj.pose);
                    
                    points_unknown = planner.plan(obj.pose, f.center, obj.map.visibility_map, 15, 1, 1);
                    points_known = planner.plan(obj.pose, f.center, obj.map.visibility_map, 1, 0, 0);
                    
                    paths = [paths;  Path(points_known)];
                    pathsUspace = [pathsUspace; Path(points_unknown)];
                    
                    if size(points_known,1) < shortestPath
                        shortestPath = size(points_known,1);
                        pathIndex = size(paths,1);
                    end
                    
                    if f_dist < closestFrontier
                        closestFrontier = f_dist;
                        frontierIndex = i;
                    end
                end
                
                temp = (FG.weights > FG.weights_uspace);
                nbf = FG.weights_uspace;
                nbf(temp==0) = Inf;
                nbf = nbf(frontierIndex,:);
                [val, idx] = min(nbf(:));
                
                u_path = [frontierIndex, idx];
                
                %%draw
                temp_map = obj.map.visibility_map;
                
                paths_fg = FG.paths_uspace;
                for i=1:size(paths_fg,1)-1
                    for j = i+1:size(paths_fg,2)
                        p = paths_fg(i,j);
                        length = size(p.waypoints,1);
                        for k = length:-1:1
                            point = p.waypoints(k,:);
                            temp_map(point(1), point(2)) = 6;
                        end
                    end
                end
                
                map_img = ind2rgb(temp_map, color_map);
                
                subplot(2,4,5)
                imshow(map_img);
                
                p = FG.paths_uspace(min(u_path(:)),max(u_path(:)));
                path = p.waypoints;
                [m,n] = size(obj.map.visibility_map);
                obj.current_path = zeros(m,n);
                for i = p.costs:-1:1
                    point = path(i,:);
                    obj.current_path(point(1), point(2)) = 3;
                end
                
                %draw(obj, obj.map);
                s_path = paths(pathIndex);
                s_path_wp = s_path.waypoints;
                for i = s_path.costs:-1:1
                    point = s_path_wp(i,:);
                    obj.pose = [point(1), point(2)];
                    obj.complete_path(point(1), point(2)) = 3;
                    obj.sense();
                    draw(obj, obj.map);
                    pause(0.01);
                end
                
                for i = 1:p.costs
                    point = path(i,:);
                    collision = planner.checkCollision(obj.map.visibility_map, point, 1);
                    if collision == 0
                        obj.pose = [point(1), point(2)];
                        obj.complete_path(point(1), point(2)) = 3;
                    else
                        break;
                    end
                    obj.sense();
                    draw(obj, obj.map);
                    pause(0.01);
                end
                
                draw(obj, obj.map);
                %}
            end
        end
        function exploreClosestBorder(obj)
            global log;
            global total_path_length;
            global color_map;
            planner = Planner(obj);
            
            calc_counter = 0;
            percent = 0;
            while 1
                calc_counter = calc_counter +1;
                
                obj.sense(); %first sense
                draw(obj, obj.map);
                free_pixels = sum(sum(obj.map.visibility_map == 3));
                old_percent = percent;
                percent = free_pixels/obj.sensor.num_free_pixels
                fprintf(log,'iteration %d, path travelled %d, percent explored %f(+%f)\n', calc_counter,total_path_length, percent, ((percent-old_percent)*100));
                
                obj.map.frontier_map.createFrontierMap(obj.pose, 0, 0);
                FG = FrontierGraph(obj.map.frontier_map.frontiers, obj.map, planner);
                paths = Path.empty;
                paths_length = [];
                paths_uspace= Path.empty;
                path_length_uspace = [];
                
                %Calculate paths from current robot pose to frontiers
                for i = 1:size(obj.map.frontier_map.frontiers, 1)
                    f = obj.map.frontier_map.frontiers(i);
                    f_dist = f.getClosestPoint(obj.pose);
                    
                    %find collission free frontier point
                    for y = -obj.robot_size:obj.robot_size
                        for x = -obj.robot_size:obj.robot_size
                            n_i = f.center(1)+y;
                            n_j = f.center(2)+x;
                            collision = planner.checkSquareCollision(obj.map.visibility_map, [n_i n_j],2*obj.robot_size, 0);
                            if collision == 0
                                new_frontier = [n_i n_j]
                            end
                        end
                    end
                    
                    %points_unknown = planner.plan(obj.pose, f.center, obj.map.visibility_map, 10, 1, 1);
                    %points_known = planner.plan(obj.pose, f.center, obj.map.visibility_map, 10, 1, 0);
                    
                    points_unknown = planner.plan(obj.pose, new_frontier, obj.map.visibility_map, 2*obj.robot_size, 1, 1);
                    points_known = planner.plan(obj.pose, new_frontier, obj.map.visibility_map, 2*obj.robot_size, 1, 0);
                    
                    
                    paths = [paths;  Path(points_known)];%
                    paths_length = [paths_length; size(points_known,1)]; %
                    paths_uspace = [paths_uspace; Path(points_unknown)];
                    path_length_uspace = [path_length_uspace; size(points_unknown, 1)];
                end
                
                temp = paths_length==0;
                
                %drive the safe path to initial position
                [~, safe_path_index] = min(paths_length);
                safe_path = paths(safe_path_index);
                s_path_wp = safe_path.waypoints;
                for i = safe_path.costs:-1:1
                    point = s_path_wp(i,:);
                    obj.pose = [point(1), point(2)];
                    total_path_length = total_path_length + 1;
                    
                    obj.complete_path(point(1), point(2)) = 3;
                    obj.sense();
                    draw(obj, obj.map);
                    pause(0.01);
                end
            end
        end
        function explore(obj, sigma, omega, theta, t_h)
            global log;
            global color_map;
            %sigma = 1;
            planner = Planner(obj);
            obj.sense(); %first sense
            obj.map.frontier_map.createFrontierMap(obj.pose, sigma, t_h);
            FG = FrontierGraph(obj.map.frontier_map.frontiers, obj.map, planner);
            explorer = Explorer(obj, omega, theta);
            frontier = explorer.getFrontier();
            draw(obj, obj.map);
            
            calc_counter = 0;
            while ~isempty(frontier)
                calc_counter = calc_counter +1;
                fprintf(log,'%d\n', calc_counter);
                %temp_map = obj.map.visibility_map;
                temp_map = obj.map.visibility_map;
                %temp_map = zeros(240,240);
                goals = [];
                starts = [];
                weights = [];
                frontiers = obj.map.frontier_map.frontiers;
                %{
                    FG.updateGraph(frontiers);
                    
                    paths = FG.paths_uspace;
                    for i=1:size(paths,1)-1
                        for j = i+1:size(paths,2)
                            p = paths(i,j);
                            length = size(p.waypoints,1);
                            for k = length:-1:1
                                point = p.waypoints(k,:);
                                temp_map(point(1), point(2)) = 6;
                            end
                        end
                    end
                    
                    map_img = ind2rgb(temp_map, color_map);
                    
                    for i = 1:size(paths,1)
                        s = sprintf('f_%d',obj.map.frontier_map.frontiers(i).id);
                        pos_y = floor(mean(obj.map.frontier_map.frontiers(i).points(:,1)));
                        pos_x = floor(mean(obj.map.frontier_map.frontiers(i).points(:,2)));
                        map_img = insertText(map_img,[pos_x pos_y],s);
                    end
                    
                    subplot(2,4,5)
                    imshow(map_img);
                %}
                goal_is_found = 0;
                while(~goal_is_found)
                    draw(obj, obj.map);
                    pause(0.01);
                    for k = 0:360
                        r = ((2*pi)/360)*k;
                        y = floor(frontier(2)+sin(r)*(2*obj.robot_size+obj.collision_radius));
                        x = floor(frontier(1)+cos(r)*(2*obj.robot_size+obj.collision_radius));
                        
                        collision = planner.checkCollision(obj.map.visibility_map, [x y], 1, 0);
                        distance = norm([x y] - obj.pose);
                        if ~collision && distance > 1.5
                            obj.goal = [x, y];
                            obj.pose;
                            goal_is_found = 1;
                            obj.collision_radius = 1;
                            break;
                        end
                    end
                    obj.collision_radius = obj.collision_radius+1;
                end
                path = planner.plan(obj.pose, obj.goal, obj.map.visibility_map, obj.robot_size+2, 1, 0);
                
                
                path = planner.plan(obj.pose, obj.goal, obj.map.visibility_map, 1, 1, 0);
                length = size(path,1);
                
                [m,n] = size(obj.map.visibility_map);
                obj.current_path = zeros(m,n);
                for i = length:-1:1
                    point = path(i,:);
                    obj.current_path(point(1), point(2)) = 3;
                end
                
                %draw(obj, obj.map);
                
                for i = length:-1:1
                    point = path(i,:);
                    obj.pose = [point(1), point(2)];
                    obj.complete_path(point(1), point(2)) = 3;
                    obj.sense();
                    draw(obj, obj.map);
                    pause(0.01);
                end
                
                obj.sense();
                obj.map.frontier_map.createFrontierMap(obj.pose, sigma, t_h);
                frontier = explorer.getFrontier();
                
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
            obj.map.visibility_map = obj.sensor.update(obj.pose, obj.map.visibility_map);
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
            end
            
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
                image = (rgb2ind(new_image, color_map));
            end
            
        end
    end
end

%{
                min = Inf;
                frontier_index = [];
                path = [];

                buff = (obj.robot_size+1)^2;
                for i = -buff:buff
                    for j = -buff:buff
                        n_i = frontier.center(1)+i;
                        n_j = frontier.center(2)+j;
                        if n_i > 0 && n_i <= m && n_j > 0 && n_j <= n 
                            if r_map(n_i, n_j) > 0
                                distance = PathPlanner.getEuklidianDistance(n_i, n_j, frontier.center(1), frontier.center(2));
                                if distance < min
                                    min = distance;
                                    frontier_index = [n_i n_j];
                                end
                            end
                        end
                    end
                end
                %}