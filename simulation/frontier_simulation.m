%colors
black = [0 0 0];
grey = [0.5 0.5 0.5];
white = [1 1 1];
red = [1 0 0];

cmap = [black; grey; white; red];

start_pose = [240 240];

path = '~/research/frontier_exploration/map.bmp';
map = Map(path, cmap);


pose = start_pose;
dist = 50;
direction = 'inc';
goal = [start_pose(1) start_pose(2) + dist];


r = Robot(10, start_pose, map, 150, cmap);

while(true)
    %update(map, pose, 100);
    
    r.moveY(1);
    
    %{
    if(direction == 'inc')
        if(pose(2) - start_pose(2) > dist)
            direction = 'dec';
        end
        pose(2) = pose(2) + 1;
    end
    
    if(direction == 'dec')
        if(pose(2) - start_pose(2) < -dist)
            direction = 'inc';
        end
        pose(2) = pose(2) - 1;
    end
    
    
    %figure(2)
    %imshow(ind2rgb(map.visibility_map, cmap))
    %colormap(cmap);
    
    %pause(0.001);
    %}
end