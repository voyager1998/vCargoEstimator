%% Initialization; load depth images
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));
addpath(strcat(pwd,'/calibration'));

depth_file='/data/boxA/DepthImage_0.png';
bias_file='calibration/bias.mat';
intrinsic_file='calibration/panasonicIRcameraParams.mat';

% load depth image
D = imread(strcat(pwd, depth_file));
% load intrinsic matrix
load(intrinsic_file);
C_ir = irCameraParams.IntrinsicMatrix';
% load bias elimination relation
bias=load(bias_file);

%% Calculate height, length, width
[dimension,image_counter]=calculate_dimension(D,irCameraParams,bias,image_counter);
fprintf('Length=%.0f mm, Width=%.0f mm, Height=%.0f mm\n',dimension(2),dimension(3),dimension(1));
vol=dimension(2)*dimension(3)*dimension(1);
fprintf('Vol=%.0f mm^3\n', vol);