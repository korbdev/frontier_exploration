black = [0 0 0];
grey = [0.5 0.5 0.5];
white = [1 1 1];
red = [1 0 0];
green = [0 1 0];
blue = [0 0 1];

global color_map;
color_map = [black; grey; white; red; green; blue];

path = '~/research/frontier_exploration/map_14.bmp';
occupancy_map_pixel = imread(path);
map = rgb2ind(occupancy_map_pixel,color_map);
[rows, cols] = size(map);
%rows = 10;
%cols = 20;
%map = zeros(rows, cols);
%map(rows/2, 1:cols/2) = 1;
p = PathPlanner(map+1, rows, cols, 15, 15, 2, true);
r = p.planCostMap();
subplot(2,2,1);
imagesc(r)
subplot(2,2,2);
imagesc(map)

%path = p.generatePath(9, 19)

%path_map = map;
%for i = 1:size(path,1)
%    path_map(path(i, 1), path(i,2)) = 10;
%end

%subplot(2,2,3);
%imagesc(path_map)