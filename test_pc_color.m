%% Initialization, load RGB and IR images
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));

RGB = imread(strcat(pwd, '/data/data0618_2/RGBImage_12.png'));
% RGB = imread(strcat(pwd, '/data/fix/fix80/RGBImage_1.png'));
% RGB = imread(strcat(pwd, '/data/dataSmall/RGBImage_6.png'));
redChannel = RGB(:, :, 1);
greenChannel = RGB(:, :, 2);
blueChannel = RGB(:, :, 3);
grayfromRGB = rgb2gray(RGB);
load('calibration/panasonicRGBcameraParams.mat');
C_rgb = rgbCameraParams.IntrinsicMatrix';
rgb_undistort = undistortImage(RGB,rgbCameraParams);
rgb_denoise= imbilatfilt(rgb_undistort, 1500, 5); % denoise

D = imread(strcat(pwd, '/data/data0618_2/DepthImage_12.png'));
% D = imread(strcat(pwd, '/data/fix/fix80/DepthImage_1.png'));
% D = imread(strcat(pwd, '/data/dataSmall/DepthImage_6.png'));
D = D/16;
load('calibration/panasonicIRcameraParams.mat');
C_ir = irCameraParams.IntrinsicMatrix';
D_undistort = undistortImage(D,irCameraParams);
D_denoise = imbilatfilt(D_undistort, 1500, 5);

%% construct color pc
% 2d pixel in tof camera's perspective -> 3d world points in tof camera's coordinate system
load('calibration/panasonicStereoParams.mat');
% pc_ir = tof2pc(D_denoise, C_ir);
[ptcloud, color] = tofRGB2pcColor(D_undistort, rgb_undistort, stereoParams);
pcshow(ptcloud, uint8(color));

