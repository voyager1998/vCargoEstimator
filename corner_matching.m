%% Initialization
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));

%% Use RANSAC to estimate ROIs for corners


%% Load RGB image; Compute corners
RGB = imread(strcat(pwd, '/data/dataSmall/RGBImage_8.png'));
grayfromRGB = rgb2gray(RGB);
load('calibration/rgb.mat');
C_rgb = rgbCameraParams.IntrinsicMatrix';
rgb_undistort = undistortImage(grayfromRGB,rgbCameraParams);

% roi_gray = [150 155 20 20];
% roi_gray = [318 238 4 4];
% corner_gray = detectMinEigenFeatures(rgb_undistort, 'ROI', roi_gray);
corner_gray = detectMinEigenFeatures(rgb_undistort);
figure(image_counter);
image_counter = image_counter + 1;
imshow(rgb_undistort)
hold on
plot(corner_gray.selectStrongest(50));


%% Load IR image; Compute corners
IR = imread(strcat(pwd, '/data/dataSmall/GrayImage_8.png'));
load('calibration/ir.mat');
C_ir = ircameraParams.IntrinsicMatrix';
IR_undistort = undistortImage(IR,ircameraParams);

% roi_ir = [260 190 20 20];
% roi_ir = [318 238 4 4];
% corner_ir = detectMinEigenFeatures(IR_undistort, 'ROI', roi_ir);
corner_ir = detectMinEigenFeatures(IR_undistort);
figure(image_counter);
image_counter = image_counter + 1;
imshow(IR_undistort)
hold on
plot(corner_ir.selectStrongest(50));

%% Re-project to 3D
pt3d_ir = dual_vision(C_ir, C_rgb, corner_ir.Location(1,:), ...
    corner_gray.Location(1,:), 35, 0)




