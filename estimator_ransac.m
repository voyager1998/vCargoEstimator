% Input: a tof depth image
% Output: possible planes in the image

%% Initialization, load depth image
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));
addpath(strcat(pwd,'/calibration'));
D = imread(strcat(pwd, '/data/data0614/DepthImage_1.png'));
D = D/16;
load('calibration/ir.mat');
C = ircameraParams.IntrinsicMatrix';
D_undistort = undistortImage(D,ircameraParams);

figure(image_counter);
image_counter = image_counter + 1;
imagesc(D_undistort)
set(gca,'dataAspectRatio',[1 1 1])
title('Depth Undistorted')

%% Construct Point Cloud
pc = tof2pc(D_undistort, C);
pc(:,1) = -pc(:,1); %TODO: what causes this flip?
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
for i = 1:numplanes
    inlier_thres = 20;
    if (i == 1) 
        inlier_thres = 30;
    end
    noise_ths = ones(1, length(pc)) * inlier_thres;
    [plane_model, outlier_ratio, plane_area, inliers, best_inliers] ...
        = ransac_fitplane(pc, 1:length(pc), noise_ths, iterations, subset_size);
    pc(best_inliers, :) = [];
    pcshow(inliers, [bitshift(bitand(i,4),-2) bitshift(bitand(i,2),-1) bitand(i,1)]);
end
title('inliers');
xlabel('X');
ylabel('Y');
zlabel('Z');
