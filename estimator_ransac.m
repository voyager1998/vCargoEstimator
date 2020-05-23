% Input: a tof depth image
% Output: possible planes in the image

clear;
addpath(['/home/',getenv('USER'),'/vCargoEstimator']);
addpath(['/home/',getenv('USER'),'/vCargoEstimator/utils']);
load('sample_input.mat');
image_counter = 1;

%% Construct Point Cloud
pc = depth2pc(D, C);
pcSize = size(pc, 1)*size(pc, 2);
figure(image_counter);
image_counter = image_counter + 1;
pcshow(PC)
title('Pointcloud')
xlabel('X')
ylabel('Y') 
zlabel('Z')
pc = reshape(pc, [pcSize, 3]);

%% RANSAC
pixel_ids = 1:pcSize;
noise_ths = ones(1, pcSize) * 2;
iterations = 100;
subset_size = 3;
[plane_model, outlier_ratio, plane_area, inliers] = ransac_fitplane(pc, pixel_ids, noise_ths, iterations, subset_size);

figure(image_counter);
image_counter = image_counter + 1;
pcshow(inliers)
title('inliers')
xlabel('X')
ylabel('Y') 
zlabel('Z')
