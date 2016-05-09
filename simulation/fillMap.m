function map = fillMap(pose, map)
    color = map(pose(1), pose(2));
    if(color == 2)
        map(pose(1), pose(2)) = 4;
        fillMap([pose(1) pose(2)+1], map);
        fillMap([pose(1) pose(2)-1], map);
        fillMap([pose(1)+1 pose(2)], map);
        fillMap([pose(1)-1 pose(2)], map);
    end
    return;
end