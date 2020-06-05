%% Initialization, load RGB and IR images
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));

RGB = imread(strcat(pwd, '/data/dataBig/RGBImage_0.png'));
figure(image_counter);
image_counter = image_counter + 1;
imagesc(RGB)
set(gca,'dataAspectRatio',[1 1 1])
title('RGB')

IRfromRGB = rgb2gray(RGB);
figure(image_counter);
image_counter = image_counter + 1;
imagesc(IRfromRGB)
set(gca,'dataAspectRatio',[1 1 1])
title('IRfromRGB')

IR = imread(strcat(pwd, '/data/dataBig/GrayImage_0.png'));
figure(image_counter);
image_counter = image_counter + 1;
imagesc(IR)
set(gca,'dataAspectRatio',[1 1 1])
title('IR')

%% Rectify
load('calibration/stereoCalibrationParam.mat');
[RGBRect, IRRect] = rectifyStereoImages(IRfromRGB, IR, stereoParams);
figure(image_counter);
image_counter = image_counter + 1;
imshow(stereoAnaglyph(RGBRect, IRRect));
title('Rectified Frames');

%% Compute Disparity
disparityMap = disparitySGM(RGBRect, IRRect);
figure(image_counter);
image_counter = image_counter + 1;
imshow(disparityMap, [0, 64]);
title('Disparity Map');
colormap jet
colorbar

%% Reconstruct the 3-D Scene
points3D = reconstructScene(disparityMap, stereoParams);
figure(image_counter);
image_counter = image_counter + 1;
pcshow(points3D)
title('Pointcloud')
xlabel('X')
ylabel('Y')
zlabel('Z')


