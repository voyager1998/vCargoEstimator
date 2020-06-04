% Input: a tof depth image
% Output: possible planes in the image

%% Initialization, load depth image
clear;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));
addpath(strcat(pwd,'/calibration'));
D = imread(strcat(pwd, '/data/0605/DepthImage_0.png'));
D = D/16;
load('calibration/fixorigincalibration.mat');
C = cameraParams.IntrinsicMatrix';

figure(image_counter);
image_counter = image_counter + 1;
imagesc(D)
set(gca,'dataAspectRatio',[1 1 1])
title('Depth')

%% Construct Point Cloud
pc = depth2pc(D, C);
pcSize = size(pc, 1)*size(pc, 2);
pc = reshape(pc, [pcSize, 3]);
figure(image_counter);
image_counter = image_counter + 1;
pcshow(pc)
title('Pointcloud')
xlabel('X')
ylabel('Y')
zlabel('Z')

%% RANSAC
pixel_ids = 1:pcSize;
noise_ths = ones(1, pcSize) * 2;
iterations = 100;
subset_size = 3;
[plane_model, outlier_ratio, plane_area, inliers] ...
    = ransac_fitplane(pc, pixel_ids, noise_ths, iterations, subset_size);

figure(image_counter);
image_counter = image_counter + 1;
pcshow(inliers)
title('inliers')
xlabel('X')
ylabel('Y') 
zlabel('Z')
