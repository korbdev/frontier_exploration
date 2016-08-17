I = imread('~/research/frontier_exploration/map_24.gif');
%I = imread('cameraman.tif');
imshow(I)
title('Original Image')

mask = false(size(I));
%mask(170,70) = true;

mask(1,1) = true;

W = graydiffweight(I, mask, 'GrayDifferenceCutoff', 25);

thresh = 0.01;
[BW, D] = imsegfmm(W, mask, thresh);
figure
imshow(BW)
title('Segmented Image')

figure
imshow(D)
title('Geodesic Distances')

imwrite(D,'~/research/frontier_exploration/output/D_24.png');