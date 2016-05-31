function p = transformCoordinatesToRobot(point_to_transform, parent)
angle = 0;
x = point_to_transform(2)*cos(angle)+point_to_transform(1)*sin(angle);
y = -point_to_transform(2)*sin(angle)+point_to_transform(1)*cos(angle);
p = [y, x]-parent;
end