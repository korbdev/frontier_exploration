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
set(0,'RecursionLimit',10000)
%fillMap([240 240], occupancy_map, cmap);
%figure(1)
%fill_img = ind2rgb(occupancy_map, cmap);
%imshow(fill_img);
       
for i = 1:50
   r.moveY(1);
   if mod(i,1) == 0
       draw(r, map, cmap);
       drawTrajectory(r, map, cmap);
   end
   pause(0.001);
end



%while(true)
%    r.moveY(1);
%    draw(r, map, cmap);
%    r.trajectory
%end