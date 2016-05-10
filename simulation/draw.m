function draw(robot, map, color_map)
    map.visibility_map(robot.pose(1), robot.pose(2)) = 4;
    rgb_img = ind2rgb(map.visibility_map, color_map);
    shapeInserter = vision.ShapeInserter('Shape','Circles');
    circle = int32([robot.pose(2) robot.pose(1) robot.robot_size]);
    new_image = step(shapeInserter, rgb_img, circle);

    %figure(1);
    %subimage(new_image)
    
    frontier_img = ind2rgb(map.frontier_map, color_map);
    %figure(1);
    %subimage(frontier_img)
    
    subplot(2,2,1), imshow(new_image)
    subplot(2,2,2), imshow(frontier_img)
end