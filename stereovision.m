%% Initialization, load RGB and IR images
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));

RGB = imread(strcat(pwd, '/data/dataBig/RGBImage_11.png'));
figure(image_counter);
image_counter = image_counter + 1;
imagesc(RGB)
set(gca,'dataAspectRatio',[1 1 1])
title('RGB')

% turn RGB to gray
IRfromRGB = rgb2gray(RGB);
figure(image_counter);
image_counter = image_counter + 1;
imagesc(IRfromRGB)
set(gca,'dataAspectRatio',[1 1 1])
title('IRfromRGB')

IR = imread(strcat(pwd, '/data/dataBig/GrayImage_11.png'));
figure(image_counter);
image_counter = image_counter + 1;
imagesc(IR)
set(gca,'dataAspectRatio',[1 1 1])
title('IR')

% scale intensity of ir and gray image
IR = histeq(IR);
figure(image_counter);
image_counter = image_counter + 1;
imagesc(IR)
set(gca,'dataAspectRatio',[1 1 1])
title('IRscaled')

IRfromRGB = histeq(IRfromRGB);
figure(image_counter);
image_counter = image_counter + 1;
imagesc(IRfromRGB)
set(gca,'dataAspectRatio',[1 1 1])
title('RGBscaled')

%% Rectify
load('calibration/stereoCalibrationParam.mat');
[RGBRect, IRRect] = rectifyStereoImages(IRfromRGB, IR, stereoParams, 'OutputView','full');
figure(image_counter);
image_counter = image_counter + 1;
imagesc(RGBRect)
set(gca,'dataAspectRatio',[1 1 1])
title('RGBRect')

figure(image_counter);
image_counter = image_counter + 1;
imagesc(IRRect)
set(gca,'dataAspectRatio',[1 1 1])
title('IRRect')

figure(image_counter);
image_counter = image_counter + 1;
imagesc(stereoAnaglyph(RGBRect, IRRect));
set(gca,'dataAspectRatio',[1 1 1])
title('Rectified Frames');

%% Compute Disparity
disparityMap = disparitySGM(RGBRect, IRRect);
figure(image_counter);
image_counter = image_counter + 1;
imagesc(disparityMap, [0, 64]);
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


