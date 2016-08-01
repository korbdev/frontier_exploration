%colors
black = [0 0 0];
grey = [0.5 0.5 0.5];
white = [1 1 1];
red = [1 0 0];
green = [0 1 0];
blue = [0 0 1];

global color_map;
color_map = [black; grey; white; red; green; blue];

path = '~/research/frontier_exploration/map_17.bmp';
occupancy_map_pixel = imread(path);
occupancy_map = rgb2ind(occupancy_map_pixel,color_map);

map = ind2rgb(occupancy_map, color_map);

map = imgaussfilt(map, 5);

BW = im2bw(map,0.5);

opts = statset('Display','final');
[idx,C] = kmeans(BW, 6);

%BW = bwmorph(BW,'thin',Inf);
%corners = detectHarrisFeatures(BW);
%[features, valid_corners] = extractFeatures(BW, corners);

%figure(1)
%imagesc(BW); hold on;
%plot(C(:,6), C(:,5),'kx', 'MarkerSize',15,'LineWidth',3)
%plot(C(:,1),C(:,2),'kx', 'MarkerSize',15,'LineWidth',3)
%hold off;
figure(1)
imshow(BW); hold on;
plot(valid_corners);
hold off;

im = -bwdist(~BW, 'chessboard');
im2 = -bwdist(~BW, 'cityblock');

wim = watershed(im, 8);
wim2 = watershed(im2, 8);

figure(2);clf;
imagesc(wim);
colorbar;

figure(3);clf;
imagesc(im);
colorbar;

figure(4);clf;
imagesc(wim2);
colorbar;