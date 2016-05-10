function drawTrajectory(robot, map, color_map)
    map.occupancy_map(robot.pose(1), robot.pose(2)) = 4;
    rgb_img = ind2rgb(map.occupancy_map, color_map);
    %shapeInserter = vision.ShapeInserter('Shape','Circles');
    %circle = int32([robot.pose(2) robot.pose(1) robot.robot_size]);
    %new_image = step(shapeInserter, rgb_img, circle);

    [m, n] = size(robot.trajectory);
    circles = [];
    for i = 1:4:m
        circles = [ circles; robot.trajectory(i,2) robot.trajectory(i,1), robot.robot_size]; 
    end
    
    shapeInserter = vision.ShapeInserter('Shape','Circles');
    new_image = step(shapeInserter, rgb_img, circles);
    
    %figure(1);
    subplot(2,2,3), imshow(new_image)
    
    %frontier_img = ind2rgb(map.frontier_map, color_map);
    %figure(3);
    %imshow(frontier_img);
end