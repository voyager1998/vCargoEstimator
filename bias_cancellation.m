%% Initialization, load depth image
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));
load('calibration/panasonicIRcameraParams.mat');
C_ir = irCameraParams.IntrinsicMatrix';

groundtruth=[50,55,60,70,75,80,85,90,95,100,105];
dist=zeros(size(groundtruth,2),4); % there are 4 depth images in each distances' file
for num=1:size(groundtruth,2)
    for idx=1:4
        filename=['/data/plane2/plane' num2str(groundtruth(num),'%d') '/DepthImage_' num2str(idx-1,'%d'), '.png'];
        D = imread(strcat(pwd, filename));
        D = D/16;
        D_undistort = undistortImage(D,irCameraParams);
        pc=tof2pc(D_undistort, C_ir);
        dist(num,idx)=plane_camera_dist(pc);
    end
end

%% compare distorted and undistorted pc
% pc_distort=tof2pc(D, C_ir);
% figure(image_counter);
% image_counter=image_counter+1;
% pcshow(pc_distort, [0 0 1])
% hold on
% pcshow(pc, [0 1 0])
% syms X Y Z
% [X,Y]=meshgrid(-1000:50:1000);
% Z1= 900+0*X;
% surf(X,Y,Z1)
% title('point cloud of distorted depth image(blue) and undistorted image(green)');

%% function to calculate distance from plane to camera
function dist = plane_camera_dist(pc)
% RANSAC fit plane from tof's pc
numplanes = 1; % one plane
iterations = 100;
subset_size = 3;

% figure(image_counter);
% hold on
plane_models = zeros(numplanes,4);
plane_points{1,numplanes} = [];
for i = 1:numplanes
    inlier_thres = 10;
    if (i == 1) 
        inlier_thres = 30;
    end
    noise_ths = ones(1, length(pc)) * inlier_thres;
    [plane_models(i,:), outlier_ratio, plane_area, inliers, best_inliers] ...
        = ransac_fitplane(pc, 1:length(pc), noise_ths, iterations, subset_size);
    pc(best_inliers, :) = [];
    plane_points{i} = inliers;
%     pcshow(inliers, [bitshift(bitand(i,4),-2) bitshift(bitand(i,2),-1) bitand(i,1)]);
end
% title('fit plane using pc from rgb');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% hold off;

%% calculate distance from camera to the plane
p1 = plane_models;
n1 = p1(1:3);
n1 = n1./norm(n1);
l = [0,0,0]-mean(plane_points{i});
dist = abs(dot(l, n1));
end

