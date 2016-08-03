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

%canny = edge(occupancy_map);

BW = ind2rgb(occupancy_map, color_map);

%map = imgaussfilt(map, 5);

%BW = im2bw(map,0.5);

%[idx,C] = kmeans(BW, 6);

%BW = bwmorph(BW(:,:,1),'dilate',Inf);
%corners = detectHarrisFeatures(BW);
%[features, valid_corners] = extractFeatures(BW, corners);

%figure(1)
%imagesc(BW); hold on;
%plot(C(:,6), C(:,5),'kx', 'MarkerSize',15,'LineWidth',3)
%plot(C(:,1),C(:,2),'kx', 'MarkerSize',15,'LineWidth',3)
%hold off; 
%hold on;
%plot(valid_corners);
%hold off;

%im = -bwdist(~BW, 'chessboard');

gauss = imgaussfilt(imcomplement(BW), 5);
%im = imgradient(gauss(:,:,1));
im = gauss;

im2 = -bwdist(~BW, 'euclidean');

figure(1)
imshow(im);

wim = watershed(im, 8);
wim2 = watershed(im2, 8);

figure(2);clf;
imshow(wim);

figure(3);clf;
imagesc(im);

figure(4);clf;
imagesc(wim2);
colorbar;