%% Initialization, load depth image
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));
load('calibration/ir.mat');
C_ir = ircameraParams.IntrinsicMatrix';

groundtruth=[50,55,65,70,75,80,85,90,95,100,105];
dist=zeros(size(groundtruth,2),4); % there are 4 depth images in each distances' file
for num=1:size(groundtruth,2)
    figure(num);
    hold on
    syms X Y Z
    [X,Y]=meshgrid(-800:50:800);
    Z= groundtruth(num)*10+0*X;
    surf(X,Y,Z)
    hold on
    for idx=1:4
        filename=['/data/plane2/plane' num2str(groundtruth(num),'%d') '/DepthImage_' num2str(idx-1,'%d'), '.png'];
        D = imread(strcat(pwd, filename));
        D = D/16;
        D_undistort = undistortImage(D,ircameraParams);
        pc=tof2pc(D_undistort, C_ir);
        dist(num,idx)=plane_camera_dist(pc, idx);
    end
    title(['fitted plane' num2str(groundtruth(num),'%d')]);
    hold off;
end
distances=[groundtruth'*10 groundtruth'*10-mean(dist,2) mean(dist,2) dist];
% fit and plot
x=distances(:,3); % practical
y=distances(:,1); % theoretical
image_counter=image_counter+num;
figure(image_counter);
image_counter=image_counter+1;
scatter(x,y,'filled')
hold on
p = polyfit(x,y,1);
x1=linspace(450, 1100);
y1=polyval(p,x1);
plot(x1,y1,'LineWidth',2)
xlabel('practical distance')
ylabel('theoretical distance')
legend('calculated point', 'fitted line')
title('relation between practical and theoretical distance from camera to plane');
save('bias.mat','p','distances');

%% apply fitted relation
% stereovision.m

%% function to calculate distance from plane to camera
function dist = plane_camera_dist(pc, idx)
% RANSAC fit plane from tof's pc
numplanes = 1; % one plane
iterations = 100;
subset_size = 3;

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
    pcshow(inliers, [bitshift(bitand(idx,4),-2) bitshift(bitand(idx,2),-1) bitand(idx,1)]);
    hold on
end

% calculate distance from camera to the plane
p1 = plane_models;
n1 = p1(1:3);
n1 = n1./norm(n1);
l = [0,0,0]-mean(plane_points{i});
dist = abs(dot(l, n1));
end

