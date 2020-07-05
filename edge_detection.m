%% Initialization
clear; 
close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));
edge_thres = 0.1;

%% Load RGB image; Compute edges
% RGB = imread(strcat(pwd, '/data/data0618_1/RGBImage_2.png'));
% % turn RGB to gray
% grayfromRGB = rgb2gray(RGB);
% 
% load('calibration/rgb.mat');
% C_rgb = rgbCameraParams.IntrinsicMatrix';
% rgb_undistort = undistortImage(grayfromRGB,rgbCameraParams);
% 
% edge_gray = edge(rgb_undistort,'canny', edge_thres);
% 
% figure(image_counter);
% image_counter = image_counter + 1;
% imshow(edge_gray)
% title('Edges in gray image')

%% Load IR image; Compute edges
% IR = imread(strcat(pwd, '/data/data0618_1/GrayImage_2.png'));
% load('calibration/ir.mat');
% C_ir = ircameraParams.IntrinsicMatrix';
% IR_undistort = undistortImage(IR,ircameraParams);
% 
% edge_ir = edge(IR_undistort,'Canny', edge_thres);
% 
% figure(image_counter);
% image_counter = image_counter + 1;
% imshow(edge_ir)
% title('Edges in IR image')

%% Load Depth image; Compute edges
D = imread(strcat(pwd, '/data/data0618_1/DepthImage_2.png'));
load('calibration/ir.mat');
C_ir = ircameraParams.IntrinsicMatrix';
D_undistort = undistortImage(D,ircameraParams);

edge_thres = 0.1;
edge_D = edge(D_undistort,'Canny', edge_thres);

figure(image_counter);
image_counter = image_counter + 1;
imshow(edge_D)
title('Edges in Depth image')
hold on
%% RANSAC edge function based on Depth image
[rows,cols] = find(edge_D == true);
edge_pts = [rows,cols];

sampleSize = 2; % number of points to sample per trial
maxDistance = 2; % max allowable distance for inliers

fitLineFcn = @(points) polyfit(points(:,2),points(:,1),1); % fit function using polyfit
evalLineFcn = ...   % distance evaluation function
  @(model, points) sum((points(:, 1) - polyval(model, points(:,2))).^2,2);

[modelRANSAC, inlierIdx] = ransac(edge_pts,fitLineFcn,evalLineFcn, ...
  sampleSize,maxDistance);
x = 0:640;
y = modelRANSAC(1)*x+modelRANSAC(2);
plot(x,y)



%% Compute intersection of edges

