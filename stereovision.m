%% Initialization, load RGB and IR images
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));

RGB = imread(strcat(pwd, '/data/dataSmall/RGBImage_8.png'));
% figure(image_counter);
% image_counter = image_counter + 1;
% imagesc(RGB)
% set(gca,'dataAspectRatio',[1 1 1])
% title('RGB')

% turn RGB to gray
grayfromRGB = rgb2gray(RGB);
% figure(image_counter);
% image_counter = image_counter + 1;
% imagesc(grayfromRGB)
% set(gca,'dataAspectRatio',[1 1 1])
% title('gray image generated from RGB')

IR = imread(strcat(pwd, '/data/dataSmall/GrayImage_8.png'));
% figure(image_counter);
% image_counter = image_counter + 1;
% imagesc(IR)
% set(gca,'dataAspectRatio',[1 1 1])
% title('IR')

% scale intensity of ir and gray image
IR = histeq(IR);
figure(image_counter);
image_counter = image_counter + 1;
imagesc(IR)
set(gca,'dataAspectRatio',[1 1 1])
title('Enhanced the contrast of IR')

grayfromRGB = histeq(grayfromRGB);
figure(image_counter);
image_counter = image_counter + 1;
imagesc(grayfromRGB)
set(gca,'dataAspectRatio',[1 1 1])
title('Enhanced the contrast of grayfromRGB')

%% Rectify based on uncalibrated stereo cameras
blobs1 = detectSURFFeatures(grayfromRGB, 'MetricThreshold', 2000);
blobs2 = detectSURFFeatures(IR, 'MetricThreshold', 2000);
figure;
imshow(grayfromRGB);
hold on;
plot(selectStrongest(blobs1, 30));
title('Thirty strongest SURF features in I1');

figure;
imshow(IR);
hold on;
plot(selectStrongest(blobs2, 30));
title('Thirty strongest SURF features in I2');

% %% Rectify based on calibration
% load('calibration/stereoCalibrationParams.mat');
% [RGBRect, IRRect] = rectifyStereoImages(grayfromRGB, IR, stereoParams, 'OutputView','full');
% figure(image_counter);
% image_counter = image_counter + 1;
% imagesc(RGBRect)
% set(gca,'dataAspectRatio',[1 1 1])
% title('grayfromRGB Rectified')
% 
% figure(image_counter);
% image_counter = image_counter + 1;
% imagesc(IRRect)
% set(gca,'dataAspectRatio',[1 1 1])
% title('IR Rectified')
% 
% figure(image_counter);
% image_counter = image_counter + 1;
% imagesc(stereoAnaglyph(RGBRect, IRRect));
% set(gca,'dataAspectRatio',[1 1 1])
% title('Rectified Frames');
% 
% %% Compute Disparity
% disparityMap = disparitySGM(RGBRect, IRRect);
% figure(image_counter);
% image_counter = image_counter + 1;
% imagesc(disparityMap, [0, 64]);
% title('Disparity Map');
% colormap jet
% colorbar
% 
% %% Reconstruct the 3-D Scene
% points3D = reconstructScene(disparityMap, stereoParams);
% figure(image_counter);
% image_counter = image_counter + 1;
% pcshow(points3D)
% title('Pointcloud')
% xlabel('X')
% ylabel('Y')
% zlabel('Z')


