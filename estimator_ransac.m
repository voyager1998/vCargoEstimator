% Input: a tof depth image
% Output: possible planes in the image

%% Initialization, load depth image
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));
addpath(strcat(pwd,'/calibration'));
D = imread(strcat(pwd, '/data/data0618_1/DepthImage_2.png'));
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
plane_points{1,4} = [];
for i = 1:numplanes
    inlier_thres = 20;
    if (i == 1) 
        inlier_thres = 30;
    end
    noise_ths = ones(1, length(pc)) * inlier_thres;
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

%% calculate intersection line
% plot the original planes
% syms X Y Z1 Z2
% p1=plane_models(1,:);
% scale1=1./p1(3); % scale factor before z to 1
% p1=scale1*p1;
% 
% p2=plane_models(2,:);
% scale2=1./p2(3);
% p2=scale2*p2;
% 
% [X,Y]=meshgrid(-500:50:500);
% Z1=-p1(1).*X-p1(2).*Y-p1(4);
% Z2=-p2(1).*X-p2(2).*Y-p2(4);
% surf(X,Y,Z1);
% hold on;
% surf(X,Y,Z2);

% find intersection line
edges=zeros(numplanes.*(numplanes-1)./2-1, 6);
k=1;
for i=1:numplanes-1
    for j=i+1:numplanes
        p1=plane_models(i,:);
        p2=plane_models(j,:);
        n1=p1(1:3);
        M1=[0,0,-p1(4)./p1(3)];
        n2=p2(1:3);
        M2=[0,0,-p2(4)./p2(3)];
        if abs((n1*n2')/(norm(n1).*norm(n2)))>0.5
            continue
        end
        [I, u, rc] = planes_intersection(n1, M1, n2, M2, 1);
        edges(k,:)=[I u]; % I point on line, u direction vector of line
        k=k+1;
        syms t
        t=-500:500;
        line = I'+t.*(u'/norm(u'));
        plot3(line(1,:),line(2,:),line(3,:),'o');
    end
end
hold off;

%% calculate intersection point
% syms x y z
% v=[x;y;z;1];
% ekv1=plane_models(1,:)*v==0;
% ekv2=plane_models(2,:)*v==0;
% ekv3=plane_models(3,:)*v==0;
% xyz = solve([ekv1, ekv2, ekv3]);

%% calculate height
figure(image_counter);
hold on;
image_counter = image_counter + 1;
pcshow(plane_points{1});
pcshow(plane_points{2});
title('parallel planes for height calculation');
height = plane_dist(plane_models(1,:), plane_models(2,:), plane_points{1}, plane_points{2});
% height = 340.7499 / 341.3144

%% calculate width and length
figure(image_counter);
image_counter = image_counter + 1;
pcshow(plane_points{2});


