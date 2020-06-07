% Input: a tof depth image
% Output: possible planes in the image

%% Initialization, load depth image
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));
addpath(strcat(pwd,'/calibration'));
D = imread(strcat(pwd, '/data/0605/DepthImage_0.png'));
D = D/16;
load('calibration/fixorigincalibration.mat');
C = cameraParams.IntrinsicMatrix';
D_undistort = undistortImage(D,cameraParams);

figure(image_counter);
image_counter = image_counter + 1;
imagesc(D)
set(gca,'dataAspectRatio',[1 1 1])
title('Depth')

figure(image_counter);
image_counter = image_counter + 1;
imagesc(D_undistort)
set(gca,'dataAspectRatio',[1 1 1])
title('Depth Undistorted')

%% Construct Point Cloud
pc = tof2pc(D_undistort, C);
figure(image_counter);
image_counter = image_counter + 1;
pcshow(pc)
title('Pointcloud')
xlabel('X')
ylabel('Y')
zlabel('Z')

originpc = pc;

%% RANSAC
pc = originpc;

iterations = 100;
subset_size = 3;

figure(image_counter);
hold on;
image_counter = image_counter + 1;
numplanes = 4;
plane_models = zeros(numplanes,4);
for i = 1:numplanes
    noise_ths = ones(1, length(pc)) * 20;
    [plane_models(i,:), outlier_ratio, plane_area, inliers, best_inliers] ...
        = ransac_fitplane(pc, 1:length(pc), noise_ths, iterations, subset_size);
    pc(best_inliers, :) = [];
    plane_points{i} = inliers;
    pcshow(inliers, [bitshift(bitand(i,4),-2) bitshift(bitand(i,2),-1) bitand(i,1)]);
end
% plane1 blue, plane2 green, plane3 cyan, plane4 red
title('inliers');
xlabel('X');
ylabel('Y');
zlabel('Z');

%% calculate height
figure(image_counter);
hold on;
image_counter = image_counter + 1;
pcshow(plane_points{1});
pcshow(plane_points{4});
title('parallel planes for height calculation');
height = plane_dist(plane_models(1,:), plane_models(4,:), plane_points{1}, plane_points{4});
% height = 340.7499 / 341.3144

%% calculate width and length
figure(image_counter);
image_counter = image_counter + 1;
pcshow(plane_points{2});


