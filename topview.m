%% Initialization, load RGB and IR images
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));

% D must be a top view
D = imread(strcat(pwd, '/data/fix/fix80/DepthImage_1.png'));
D = D/16;
load('calibration/panasonicIRcameraParams.mat');
C_ir = irCameraParams.IntrinsicMatrix';
D_undistort = undistortImage(D,irCameraParams);
D_denoise = imbilatfilt(D_undistort, 1500, 5);

pc_ir = tof2pc(D_denoise, C_ir);

%% eliminate bias
% p=load('bias.mat').p; % bias transformation calculated from bias_cancellation.m 
% pc_ir(:,3)=polyval(p,pc_ir(:,3));

%% RANSAC fit plane from tof's pc
pc = pc_ir;

numplanes = 2; % fit 2 planes: top plane and ground
iterations = 100;
subset_size = 3;

ransac_figure = image_counter;
image_counter = image_counter + 1;
figure(ransac_figure);
hold on
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
    pcshow(inliers, [bitshift(bitand(i,4),-2) bitshift(bitand(i,2),-1) bitand(i,1)]);
end
% plane1 blue, plane2 green, plane3 cyan, plane4 red
title('fit plane using pc');
xlabel('X');
ylabel('Y');
zlabel('Z');
hold off;

%% find edge from depth image
edge_thres = 0.05;
edge_depth = edge(D_denoise,'Canny', edge_thres);

% Project points of the fitted plane from 3d pc to 2d
I = irCameraParams.Intrinsics;
edge_figure = image_counter;
image_counter = image_counter + 1;
figure(edge_figure);
imshow(edge_depth)
hold on
for i=1:1:numplanes
    DbyRGB = worldToImage(I,eye(3,3),zeros(3,1),plane_points{i});
    plot(DbyRGB(:,1), DbyRGB(:,2), '.', 'LineWidth', 1, 'MarkerSize', 1);
end
title("edges and planes of depth");
hold off;

%% RANSAC fit edge in depth image
numlines = 4; % 4 edges of a rectangle
edge_image = edge_depth;

[rows,cols] = find(edge_image == true);
ROI=(rows>100);
edge_pts = [rows(ROI),cols(ROI)];
line_2dmodels=zeros(numlines,2);
k=1;
figure(edge_figure);
hold on
for i=1:numlines
    sampleSize = 2; % number of points to sample per trial
    maxDistance = 200; % max allowable distance for inliers

    fitLineFcn = @(points) polyfit(points(:,2),points(:,1),1); % fit function using polyfit
    evalLineFcn = ...   % distance evaluation function
      @(model, points) sum((points(:, 1) - polyval(model, points(:,2))).^2,2);

    [modelRANSAC, inlierIdx] = ransac(edge_pts,fitLineFcn,evalLineFcn, ...
      sampleSize,maxDistance);
    line_2dmodels(k,:)=modelRANSAC;
    edge_pts(inlierIdx==1,:)=[];
    x = 0:640;
    y = modelRANSAC(1)*x+modelRANSAC(2);
    plot(x,y,'LineWidth',2)
    k=k+1;
end
xlim([0 640]);
ylim([0 480]);
legend({},'Location','southwest');
hold off;

%% 2d edge to 3d line
edge_3dmodels=zeros(numlines,6); % (1:3)=I,point on line % (4:6)=u,direction vector of line

figure(ransac_figure);
hold on;

p1=plane_models(2,:); % top plane
n1=p1(1:3);
M1=[0,0,-p1(4)./p1(3)];

for i=1:numlines % assume 2 lines are of interest
    pm = line2dTplane(line_2dmodels(i,:), C_ir);
    n2=pm(1:3);
    M2=[0,0,-pm(4)./pm(3)];
    
    [I, u, rc] = planes_intersection(n1, M1, n2', M2, 1);
    edge_3dmodels(i,:)=[I u]; % I point on line, u direction vector of line
    syms t
    t=-500:500;
    line = I'+t.*(u'/norm(u'));
    plot3(line(1,:),line(2,:),line(3,:),'.','Color','magenta');
end
hold off;

%% Calculate height/length/width
height = plane_dist(plane_models(1,:), plane_models(2,:), plane_points{1}, plane_points{2});

corners=zeros(numlines.*(numlines-1),3);
c=1;

figure(ransac_figure);
hold on
for i=1:numlines-1
    for j=i+1:numlines
        syms k1 k2
        p1=edge_3dmodels(i,1:3)';
        u1=edge_3dmodels(i,4:6)';
        p2=edge_3dmodels(j,1:3)';
        u2=edge_3dmodels(j,4:6)';
        if (abs(dot(u1,u2))<0.5) % perpendicular lines
            ekv1 = k1 - k2*dot(u1,u2) == dot(u1,p2-p1);
            ekv2 = k1*dot(u1,u2) - k2 == dot(u2,p2-p1);
            ret=solve([ekv1,ekv2]);

            k1=ret.k1;
            k2=ret.k2;
            intercept1=p1+k1*u1;
            intercept1=double(intercept1);
            corners(c,:)=intercept1';
            plot3(intercept1(1),intercept1(2),intercept1(3),'.','MarkerSize',40,'Color',[bitshift(bitand(c,4),-2) bitshift(bitand(c,2),-1) bitand(c,1)]);
            c=c+1;
        end
    end
end
hold off;

c=c-1;
distances=zeros(c*(c-1),1);
k=1;
for i=1:c-1
    for j=i+1:c
        distances(k)=norm(corners(i,:)-corners(j,:));
        k=k+1;
    end
end
% point order: blue grenn cyan red magenta yellow
% length=(301+315)/2=308
% width=(294+295)/2=294