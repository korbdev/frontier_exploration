function draw(robot, map)
    global color_map;
    global robot_image;
    global calc_counter;
    subplot(2, 4, 3);
    imshow(robot_image);
    
    rgb_img = ind2rgb(map.visibility_map, color_map);

    image = map.visibility_map;
    h = figure(1);
    
    subplot(2,4,1);
    
    image = robot.drawRobot(h, image);
    rgb_img = ind2rgb(image, color_map);
    imshow(rgb_img);
    
    occupancy_map_s = sprintf('~/research/frontier_exploration/simulation/occupancy_map/occupancy_map_%d.png', calc_counter);
    imwrite(rgb_img, occupancy_map_s);
    
    robot_image = rgb_img;
    
    hold on;
    [x, y] = pol2cart(robot.orientation, robot.robot_size);
    plot([robot.pose(2) robot.pose(2)+x],[robot.pose(1) robot.pose(1)+y]);
    hold off;
    
    subplot(2,4,2)
    map_img = ind2rgb(robot.sensor.occupancy_map-1, color_map);
    imshow(map_img);
    
    %subplot(2,3,3)%, imshow(frontier_img);
    %map.frontier_map.draw();
    
    %if ~isempty(map.frontier_map.theta_list)
    %    [tout, rout] = rose(map.frontier_map.theta_list, map.frontier_map.bins);
    %    subplot(2,4,3), polar(tout, rout), set(gca,'View',[0 -90]);
    %end
end