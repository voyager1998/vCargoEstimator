%% Initialization
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));

%% Use RANSAC to estimate ROIs for corners


%% Load RGB image; Compute corners
RGB = imread(strcat(pwd, '/data/dataSmall/RGBImage_8.png'));

% turn RGB to gray
grayfromRGB = rgb2gray(RGB);
% C_gray = detectHarrisFeatures(grayfromRGB);
roi_gray = [150 155 20 20];
C_gray = detectMinEigenFeatures(grayfromRGB, 'ROI', roi_gray);
figure(image_counter);
image_counter = image_counter + 1;
imshow(grayfromRGB)
hold on
plot(C_gray.selectStrongest(50));


%% Load IR image; Compute corners
IR = imread(strcat(pwd, '/data/dataSmall/GrayImage_8.png'));
% C_ir = detectHarrisFeatures(IR);
roi_ir = [260 190 20 20];
C_ir = detectMinEigenFeatures(IR, 'ROI', roi_ir);
figure(image_counter);
image_counter = image_counter + 1;
imshow(IR)
hold on
plot(C_ir.selectStrongest(50));

%% Re-project to 3D
