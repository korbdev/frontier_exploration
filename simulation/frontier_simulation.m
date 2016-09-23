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

global frontier_memory;
global saved_frontier_memory;
memory = load('memory.mat');
saved_frontier_memory = memory.frontier_memory;
frontier_memory = [];
save('memory.mat', 'frontier_memory');

global total_path_length;
global robot_image;
total_path_length = 0;
robot_image = 0;
%sigma = 1;
%omega = 0.7;
%theta = 0.3;
%t_h = 0.02;

sigma = 1;
omega = 0.5;
theta = 0.5;
t_h = 0.01;

robot_radius = 2;

%fprintf(log, 'sigma %d, omega %d, theta %d, t_h %d\n', sigma, omega, theta, t_h);

path = '~/research/frontier_exploration/map_19.bmp';
occupancy_map_pixel = imread(path);
occupancy_map = rgb2ind(occupancy_map_pixel,color_map);

[m, n] = size(occupancy_map);
sensor = Sensor(occupancy_map,robot_radius,30, pi);
map = Map(m, n);

%r = Robot(robot_radius, [20 20], sensor, map); %map 11
%r = Robot(robot_radius, [170 150], sensor, map); %map 11
%r = Robot(robot_radius, [20 120], sensor, map);
r = Robot(robot_radius, [180 180], sensor, map);
%r = Robot(2, [210 210], sensor, map);

%r = Robot(2, [160 22], sensor, map);
%r = Robot(2, [40 220], sensor, map);
%r.explore(sigma, omega, theta, t_h);
%r.exploreUspace();
%r.exploreCloseFrontiers3DSegmented();
%r.exploreCloseFrontiers3D();
r.exploreFrontierTree(0);
%r.exploreClosestBorder();

%planner = Planner(r);
%G = [ 0 80 10 120 800 1000; Inf 0 40 50 60 1000; Inf Inf 0 70 20 1000; Inf Inf Inf 0 400 1000; Inf Inf Inf Inf 0 5; Inf Inf Inf Inf Inf 0];
%G = triu(G)'+triu(G)

%planner.planGraph(G, 1, 6);