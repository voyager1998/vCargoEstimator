%% Initialization, load RGB and IR images
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));

RGB = imread(strcat(pwd, '/data/data0618_1/RGBImage_0.png'));
grayfromRGB = rgb2gray(RGB);
load('calibration/rgbCameraParams0716.mat');
C_rgb = rgbCameraParams.IntrinsicMatrix';
rgb_undistort = undistortImage(grayfromRGB,rgbCameraParams);
rgb_denoise= imbilatfilt(rgb_undistort, 1500, 5); % denoise

D = imread(strcat(pwd, '/data/data0618_1/DepthImage_0.png'));
D = D/16;
load('calibration/irCameraParams0716.mat');
C_ir = irCameraParams.IntrinsicMatrix';
D_undistort = undistortImage(D,irCameraParams);
D_denoise = imbilatfilt(D_undistort, 1500, 5);
pc_ir = tof2pc_mat(D_denoise, C_ir);

IR = imread(strcat(pwd, '/data/data0618_1/GrayImage_0.png'));
IR_undistort = undistortImage(IR,irCameraParams);
IR_denoise = imbilatfilt(IR_undistort, 1500, 5);

%% Alignment in 3d
load('calibration/stereo0718.mat');
ir_reproject = (stereoParams.RotationOfCamera2*pc_ir')'+stereoParams.TranslationOfCamera2;
figure(image_counter);
image_counter = image_counter + 1;
[ptcloud, color] = tofRGB2pcColor(D_denoise, rgb_denoise, stereoParams);
pcshow(ptcloud, uint8(color));

%% RANSAC fit plane from tof's pc
% p=load('bias.mat').p; % bias transformation calculated from bias_cancellation.m 
% pc_ir(:,3)=polyval(p,pc_ir(:,3));
% pc = pc_ir;
% 
% numplanes = 2;
% iterations = 100;
% subset_size = 3;
% 
% ransac_figure = image_counter;
% figure(ransac_figure);
% hold on;
% image_counter = image_counter + 1;
% plane_models = zeros(numplanes,4);
% plane_points{1,4} = [];
% for i = 1:numplanes
%     inlier_thres = 10;
%     if (i == 1) 
%         inlier_thres = 30;
%     end
%     noise_ths = ones(1, length(pc)) * inlier_thres;
%     [plane_models(i,:), outlier_ratio, plane_area, inliers, best_inliers] ...
%         = ransac_fitplane(pc, 1:length(pc), noise_ths, iterations, subset_size);
%     pc(best_inliers, :) = [];
%     plane_points{i} = inliers;
%     pcshow(inliers, [bitshift(bitand(i,4),-2) bitshift(bitand(i,2),-1) bitand(i,1)]);
% end
% % plane1 blue, plane2 green, plane3 cyan, plane4 red
% title('fit plane using pc from rgb');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% hold off;

% Align in 2d
% DbyRGB = worldToImage(rgbCameraParams, stereoParams.RotationOfCamera2, ...
%     stereoParams.TranslationOfCamera2,plane_points{2});
% figure(image_counter);
% image_counter = image_counter + 1;
% scatter(DbyRGB(:,1),DbyRGB(:,2),5,'filled');
% title("reproject 2d prom rgb camera's perspective");

%% Rectify based on calibration
load('calibration/stereo0718.mat');
[RGBRect, IRRect] = rectifyStereoImages(imadjust(grayfromRGB),...
    imadjust(IR_denoise), stereoParams, 'OutputView','full');
figure(image_counter);
image_counter = image_counter + 1;
imagesc(RGBRect)
set(gca,'dataAspectRatio',[1 1 1])
title('grayfromRGB Rectified')

figure(image_counter);
image_counter = image_counter + 1;
imagesc(IRRect)
set(gca,'dataAspectRatio',[1 1 1])
title('IR Rectified')

figure(image_counter);
image_counter = image_counter + 1;
imagesc(stereoAnaglyph(RGBRect, IRRect));
set(gca,'dataAspectRatio',[1 1 1])
title('Rectified Frames');

%% Compute Disparity
disparityMap = disparitySGM(RGBRect, IRRect);
figure(image_counter);
image_counter = image_counter + 1;
imagesc(disparityMap, [0, 64]);
title('Disparity Map');
colormap jet
colorbar

%% Reconstruct the 3-D Scene
points3D = reconstructScene(disparityMap, stereoParams);
figure(image_counter);
image_counter = image_counter + 1;
pcshow(points3D)
title('Pointcloud')
xlabel('X')
ylabel('Y')
zlabel('Z')
