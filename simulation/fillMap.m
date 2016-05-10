function map = fillMap(pose, map, cmap)
    [m, n] = size(map);
    if pose(1) == 1 || pose(1) == m
        return;
    end
    if pose(2) == 1 || pose(2) == n
        return;
    end
    color = map(pose(1), pose(2));
    
    if color == 2
       map(pose(1), pose(2)) = 4;
       
       %figure(1)
       %fill_img = ind2rgb(map, cmap);
       %imshow(fill_img);
       %tic;pause(0.001);toc;
       
       fillMap([pose(1) pose(2)+1], map, cmap);
       fillMap([pose(1) pose(2)-1], map, cmap);
       fillMap([pose(1)+1 pose(2)], map, cmap);
       fillMap([pose(1)-1 pose(2)], map, cmap);
    end
    return;
end