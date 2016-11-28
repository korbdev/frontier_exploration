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

global total_path_length;
global robot_image;
global calc_counter;
calc_counter = 0;
total_path_length = 0;
robot_image = 0;


sigma = 1;
omega = 0.5;
theta = 0.5;
t_h = 0.01;

robot_radius = 2;

path = '~/research/frontier_exploration/map_19.bmp';
occupancy_map_pixel = imread(path);
occupancy_map = rgb2ind(occupancy_map_pixel,color_map);

[m, n] = size(occupancy_map);
sensor = Sensor(occupancy_map,robot_radius,30, pi);
map = Map(m, n);

r = Robot(robot_radius, [180 180], sensor, map);

r.exploreCloseFrontiers3D();
