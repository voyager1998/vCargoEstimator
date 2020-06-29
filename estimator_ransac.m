% Input: a tof depth image
% Output: possible planes in the image
clear; close all; clc;
load('./data/ir.mat');
C = cameraParams.IntrinsicMatrix';
I = imread('./data/GrayImage_4.png');
I = double(I);
D = imread('./data/DepthImage_4.png');
D = double(D)/16;
D = undistortImage(D,cameraParams);
I = undistortImage(I,cameraParams);
%D= imbilatfilt(D, 1500, 5);
%D = denoise(I,D);
image_counter = 1;
figure(image_counter);
image_counter = image_counter + 1;
imagesc(D);
set(gca,'dataAspectRatio',[1 1 1])
title('Depth Image')
%% Construct Point Cloud
pc = tof2pc(D, C);
figure(image_counter);
image_counter = image_counter + 1;
pcshow(pc)
title('Pointcloud')
xlabel('X')
ylabel('Y') 
zlabel('Z')
%% RANSAC
pc = [pc zeros(size(pc,1),1)];
iterations = 100;
subset_size = 3;
figure(image_counter);
hold on;
image_counter = image_counter + 1;
numplanes = 4;
models = zeros(4,numplanes);
for i = 1:numplanes
    noise_ths = 20;
    pc_fit = pc(pc(:,4)==0,1:3);
    [plane_model, inliers, inlier_indexs] ...
        = ransac_fitplane(pc_fit, noise_ths, iterations, subset_size, models(1:3,1:i-1));
    models(:,i) = plane_model;
    pc(inlier_indexs, 4) = i;
    pc = sortrows(pc,4);
    pcshow(inliers, [bitshift(bitand(i,4),-2) bitshift(bitand(i,2),-1) bitand(i,1)]);
end
[l, w, h] = Cal_dis(pc,models);
fprintf("Result: length: %f, width: %f, height: %f \n",l,w,h);
fprintf("Truth: length: 433, width: 311, height: 281 \n");
title('inliers');
xlabel('X');
ylabel('Y');
zlabel('Z');