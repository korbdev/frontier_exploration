%colors
black = [0 0 0];
grey = [0.5 0.5 0.5];
white = [1 1 1];
red = [1 0 0];
green = [0 1 0];
blue = [0 0 1];

cmap = [black; grey; white; red; green; blue];
path = '~/research/frontier_exploration/map_4.bmp';
occupancy_map_pixel = imread(path);
occupancy_map = rgb2ind(occupancy_map_pixel,cmap);

[m, n] = size(occupancy_map);
sensor = Sensor(occupancy_map, 50);
map = Map(m, n);

r = Robot(3, [20 120], sensor, map);
r.explore();
%{
while true
    path = r.moveToFrontier();
    length = size(path,1);
    draw(r, map);
    for i = length:-1:1
       point = path(i,:);
       
       map.visibility_map(point(1), point(2)) = 4;
       r.pose = [point(1), point(2)];
       r.sense();
       draw(r, map);
       pause(0.01);
    end
    draw(r, map);
end
%}