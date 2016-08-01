function draw(robot, map)
    global color_map;
    
    rgb_img = ind2rgb(map.visibility_map, color_map);

    image = map.visibility_map;
    h = figure(1);
    
    subplot(2,4,1);
    
    image = robot.drawRobot(h, image);
    rgb_img = ind2rgb(image, color_map);
    imshow(rgb_img);

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