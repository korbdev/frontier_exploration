function draw(robot, map)
    %map.visibility_map(robot.pose(1), robot.pose(2)) = 4;
    
    %rgb_img = ind2rgb(map.visibility_map, color_map);
    %shapeInserter = vision.ShapeInserter('Shape','Circles');
    
    %circle = int32([robot.pose(2) robot.pose(1) robot.robot_size]);
    %new_image = step(shapeInserter, rgb_img, circle);

    %colors
    black = [0 0 0];
    grey = [0.5 0.5 0.5];
    white = [1 1 1];
    red = [1 0 0];

    color_map = [black; grey; white; red];
    
    map.visibility_map(robot.pose(1), robot.pose(2)) = 4;
    
    if(robot.goal)
        map.visibility_map(robot.goal(1), robot.goal(2)) = 4;
    end
    
    if(robot.frontier_point)
        map.visibility_map(robot.frontier_point(1), robot.frontier_point(2)) = 4;
    end
    
    rgb_img = ind2rgb(map.visibility_map, color_map);
    shapeInserter = vision.ShapeInserter('Shape','Circles');

    circle = int32([robot.pose(2) robot.pose(1) robot.robot_size; robot.goal(2) robot.goal(1) robot.robot_size; robot.frontier_point(2) robot.frontier_point(1) robot.robot_size*2+robot.collision_radius]);
    new_image = step(shapeInserter, rgb_img, circle);
    %subimage(new_image)
    
    frontier_img = ind2rgb(map.frontier_map, color_map);
    %figure(1);
    %subimage(frontier_img)
    
    figure(1)
    subplot(2,2,1), imshow(new_image)
    subplot(2,2,2), imshow(frontier_img);
    [tout, rout] = rose(map.frontier_polar, 360);
    subplot(2,2,3), polar(tout, rout), set(gca,'View',[90 90]);
            %set(gca,'View',[90 90]);
    %subplot(1,1,1), imshow(rgb_img)
end