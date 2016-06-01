%colors
black = [0 0 0];
grey = [0.5 0.5 0.5];
white = [1 1 1];
red = [1 0 0];
green = [0 1 0];
blue = [0 0 1];

global color_map;
color_map = [black; grey; white; red; green; blue];

global log;
log = fopen('~/research/frontier_exploration/simulation/log.txt', 'wt');

sigma = 2;
omega = 0.3;
theta = 0.7;
t_h = 0.05;

fprintf(log, 'sigma %d, omega %d, theta %d, t_h %d\n', sigma, omega, theta, t_h);

path = '~/research/frontier_exploration/map_4.bmp';
occupancy_map_pixel = imread(path);
occupancy_map = rgb2ind(occupancy_map_pixel,color_map);

[m, n] = size(occupancy_map);
sensor = Sensor(occupancy_map,30);
map = Map(m, n);

r = Robot(1, [120 120], sensor, map);
r.explore(sigma, omega, theta, t_h);