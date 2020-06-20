%% Initialization
clear; 
close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));
edge_thres = 0.1;


%% Load RGB image; Compute edges
RGB = imread(strcat(pwd, '/data/data0618_1/RGBImage_2.png'));
% turn RGB to gray
grayfromRGB = rgb2gray(RGB);

load('calibration/rgb.mat');
C_rgb = rgbCameraParams.IntrinsicMatrix';
rgb_undistort = undistortImage(grayfromRGB,rgbCameraParams);

edge_gray = edge(rgb_undistort,'canny', edge_thres);

figure(image_counter);
image_counter = image_counter + 1;
imagesc(edge_gray)
title('Edges in gray image')

%% Load IR image; Compute edges
IR = imread(strcat(pwd, '/data/data0618_1/GrayImage_2.png'));
load('calibration/ir.mat');
C_ir = ircameraParams.IntrinsicMatrix';
IR_undistort = undistortImage(IR,ircameraParams);

edge_ir = edge(IR_undistort,'Canny', edge_thres);

figure(image_counter);
image_counter = image_counter + 1;
imagesc(edge_ir)
title('Edges in IR image')

%% Compute intersection of edges

