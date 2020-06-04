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
% pcSize = size(pc, 1)*size(pc, 2);
pcSize = size(pc, 1);
pc = reshape(pc, [pcSize, 3]);
ptCloud = pointCloud(pc);
% ptCloud = pcdenoise(ptCloud0);
figure(image_counter);
image_counter = image_counter + 1;
pcshow(ptCloud)
title('Pointcloud')
xlabel('X')
ylabel('Y')
zlabel('Z')

%% RANSAC
% maxDistance = 10;
% % referenceVector = [0,0,1];
% % maxAngularDistance = 5;
% % [model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud,...
% %             maxDistance,referenceVector,maxAngularDistance);
% [model1,inlierIndices,outlierIndices] = pcfitplane(ptCloud, maxDistance);
% plane1 = select(ptCloud,inlierIndices);
% figure(image_counter);
% image_counter = image_counter + 1;
% pcshow(plane1)
% title('inliers1')
% xlabel('X')
% ylabel('Y') 
% zlabel('Z')
% xlim([-600 300])
% ylim([-600 1000])
% zlim([0 1000])
% 
% PtCloud2 = select(ptCloud,outlierIndices);
% % [model2,inlierIndices,outlierIndices] = pcfitplane(PtCloud2, maxDistance);
% % plane2 = select(PtCloud2,inlierIndices);
% % figure(image_counter);
% % image_counter = image_counter + 1;
% % pcshow(plane2)
% % title('inliers2')
% % xlabel('X')
% % ylabel('Y') 
% % zlabel('Z')
% % xlim([-600 300])
% % ylim([-600 1000])
% % zlim([0 1000])

%% RANSAC custom
pixel_ids = 1:pcSize;
noise_ths = ones(1, pcSize) * 20;
iterations = 100;
subset_size = 3;
[plane_model, outlier_ratio, plane_area, inliers, outliers] ...
    = ransac_fitplane(pc, pixel_ids, noise_ths, iterations, subset_size);

figure(image_counter);
image_counter = image_counter + 1;
pcshow(inliers)
title('inliers')
xlabel('X')
ylabel('Y') 
zlabel('Z')

figure(image_counter);
image_counter = image_counter + 1;
pcshow(outliers)
title('remaining pc')
xlabel('X')
ylabel('Y') 
zlabel('Z')
