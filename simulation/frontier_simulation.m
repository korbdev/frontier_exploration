%colors
black = [0 0 0];
grey = [0.5 0.5 0.5];
white = [1 1 1];
red = [1 0 0];

cmap = [black; grey; white; red];
path = '~/research/frontier_exploration/map.bmp';
occupancy_map_pixel = imread(path);
occupancy_map = rgb2ind(occupancy_map_pixel,cmap);

map = Map(occupancy_map);

r = Robot(10, [240 240], map, 100);

%planner = Planner(r);
%path = planner.plan([240 240],[410 60], occupancy_map);

%[m, n] = size(occupancy_map);
%path_ind = ones(m,n);

%path = r.moveTo([420 100]);

while true
    path = r.moveToFrontier();
    length = size(path,1);
    for i = length:-1:1
       point = path(i,:);
       
       %occupancy_map(point(1), point(2)) = 4;
       r.pose = [point(1), point(2)];
       sense(r);
       draw(r, map);
       pause(0.01);
       %drawTrajectory(r, map, cmap);
    end
    draw(r, map);
    pause(0.001);
end

%{
length = size(path,1);
for i = length:-1:1
   point = path(i,:);
   occupancy_map(point(1), point(2)) = 4;
   r.pose = [point(1), point(2)];
   draw(r, map, cmap);
   pause(0.01);
   %drawTrajectory(r, map, cmap);
end

path = r.moveTo([400 400]);

length = size(path,1);
for i = length:-1:1
   point = path(i,:);
   occupancy_map(point(1), point(2)) = 4;
   r.pose = [point(1), point(2)];
   draw(r, map, cmap);
   pause(0.01);
   %drawTrajectory(r, map, cmap);
end

path = r.moveTo([100 400]);

length = size(path,1);
for i = length:-1:1
   point = path(i,:);
   occupancy_map(point(1), point(2)) = 4;
   r.pose = [point(1), point(2)];
   draw(r, map, cmap);
   pause(0.01);
   %drawTrajectory(r, map, cmap);
end

path = r.moveTo([160 50]);

length = size(path,1);
for i = length:-1:1
   point = path(i,:);
   occupancy_map(point(1), point(2)) = 4;
   r.pose = [point(1), point(2)];
   draw(r, map, cmap);
   pause(0.01);
   %drawTrajectory(r, map, cmap);
end

path = r.moveTo([420 80]);

length = size(path,1);
for i = length:-1:1
   point = path(i,:);
   occupancy_map(point(1), point(2)) = 4;
   r.pose = [point(1), point(2)];
   draw(r, map, cmap);
   pause(0.01);
   %drawTrajectory(r, map, cmap);
end
%}

%path_img = ind2rgb(occupancy_map, cmap);
%imshow(path_img);
%fillMap([240 240], occupancy_map, cmap);
%figure(1)
%fill_img = ind2rgb(occupancy_map, cmap);
%imshow(fill_img);
       
%for i = 1:50
%   r.moveY(1);
%   if mod(i,1) == 0
%       draw(r, map, cmap);
%       drawTrajectory(r, map, cmap);
%   end
%   pause(0.001);
%end



%while(true)
%    r.moveY(1);
%    draw(r, map, cmap);
%    r.trajectory
%end