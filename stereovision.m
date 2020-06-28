%% Initialization, load RGB and IR images
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));

RGB = imread(strcat(pwd, '/data/data0618_1/RGBImage_7.png'));
grayfromRGB = rgb2gray(RGB);
load('calibration/panasonicRGBcameraParams.mat');
C_rgb = rgbCameraParams.IntrinsicMatrix';
rgb_undistort = undistortImage(grayfromRGB,rgbCameraParams);
rgb_denoise= imbilatfilt(rgb_undistort, 1500, 5); % denoise

D = imread(strcat(pwd, '/data/data0618_1/DepthImage_7.png'));
D = D/16;
load('calibration/panasonicIRcameraParams.mat');
C_ir = irCameraParams.IntrinsicMatrix';
D_undistort = undistortImage(D,irCameraParams);
D_deoise = imbilatfilt(D_undistort, 1500, 5);

% 2d pixel in tof camera's perspective -> 3d world points in tof camera's coordinate system
pc_ir = tof2pc(D_deoise, C_ir);

%% 3d world points in tof's coordinate system -> rgb's coordinate system
load('calibration/panasonicStereoParams.mat');
pc_rgb = pc_ir*stereoParams.RotationOfCamera2+stereoParams.TranslationOfCamera2;
figure(image_counter);
pc_figure = image_counter;
image_counter = image_counter + 1;
pcshow(pc_ir,[0 0 1]);
hold on;
pcshow(pc_rgb,[0 1 0]);
title("pc from the perspective of both rgb[green] and tof[blue] camera");
hold off;

% 3d pc -> 2d pixel in rgb camera's perspective
I = rgbCameraParams.Intrinsics;
DbyRGB = worldToImage(I,eye(3,3),zeros(3,1),pc_rgb);
figure(image_counter);
image_counter = image_counter + 1;
scatter(DbyRGB(:,1),DbyRGB(:,2),5,'filled');
title("reproject 2d prom rgb camera's perspective");

%% RANSAC fit plane from rgb's pc
pc = pc_rgb;

iterations = 100;
subset_size = 3;

ransac_figure = image_counter;
figure(ransac_figure);
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
title('fit plane using pc from rgb');
xlabel('X');
ylabel('Y');
zlabel('Z');
hold off;

%% find intersection line of every 2 planes
figure(ransac_figure);
hold on;

line_3dmodels=zeros(numplanes.*(numplanes-1)./2-1,6); % (1:3)=I,point on line % (4:6)=u,direction vector of line
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
        line_3dmodels(k,:)=[I u]; % I point on line, u direction vector of line
        k=k+1;
        syms t
        t=-500:500;
        line = I'+t.*(u'/norm(u'));
        plot3(line(1,:),line(2,:),line(3,:),'.','Color','y');
    end
end
hold off;

%% find edge from rgb image
edge_thres = 0.1;
edge_rgb = edge(rgb_denoise,'Canny', edge_thres);
figure(image_counter);
edge_figure = image_counter;
image_counter = image_counter + 1;
imagesc(edge_rgb);
title('edge detected in rgb image');

%% RANSAC fit edge in rgb image
numlines = 7;
[rows,cols] = find(edge_rgb == true);
edge_pts = [rows,cols];
line_2dmodels=zeros(numlines,2);
k=1;
% line_points{1,numlines}=[];
figure(edge_figure);
hold on;
for i=1:numlines
    sampleSize = 2; % number of points to sample per trial
    maxDistance = 2; % max allowable distance for inliers

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
legend({'1','2','3','4','5','6','7'},'Location','southwest');
hold off;

%% From edge to plane; Compute plane intersection
edge_3dmodels=zeros(numlines,6); % (1:3)=I,point on line % (4:6)=u,direction vector of line

figure(ransac_figure);
hold on;

p1=plane_models(2,:); % top plane
n1=p1(1:3);
M1=[0,0,-p1(4)./p1(3)];

for i=1:numlines % assume 2 lines are of interest
    pm = line2dTplane(line_2dmodels(i,:), C_rgb);
    n2=pm(1:3);
    M2=[0,0,-pm(4)./pm(3)];
    
    [I, u, rc] = planes_intersection(n1, M1, n2', M2, 1);
    edge_3dmodels(i,:)=[I u]; % I point on line, u direction vector of line
    syms t
    t=-500:500;
    line = I'+t.*(u'/norm(u'));
    plot3(line(1,:),line(2,:),line(3,:),'.','Color','magenta');
end

% plot the camera's view plane through the above edge
% syms X Y Z1
% scale1=1./pm(3); % scale factor before z to 1
% pm=scale1*pm;
% 
% [X,Y]=meshgrid(-400:50:400);
% Z1=-pm(1).*X-pm(2).*Y-pm(4);
% surf(X,Y,Z1);

%% Calculate length and length
corners=zeros(numlines.*(numlines-1),3);
c=1;

figure(ransac_figure);
hold on;
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
            % intercept2=p2+k2*u2;
            % intercept2=double(intercept2);
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
% blue grenn cyan red magenta yellow
% length=(466.0544+445.7806)/2=455.9175
% width=(278.3207+283.8947)/2=281.1077


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Rectify based on uncalibrated stereo cameras

% blobs1 = detectSURFFeatures(grayfromRGB, 'MetricThreshold', 2000);
% blobs2 = detectSURFFeatures(IR, 'MetricThreshold', 2000);
% figure;
% imshow(grayfromRGB);
% hold on;
% plot(selectStrongest(blobs1, 30));
% title('Thirty strongest SURF features in I1');
% 
% figure;
% imshow(IR);
% hold on;
% plot(selectStrongest(blobs2, 30));
% title('Thirty strongest SURF features in I2');

%% Rectify based on calibration
% load('calibration/panasonicStereoParams.mat');
% [RGBRect, IRRect] = rectifyStereoImages(grayfromRGB, IR, stereoParams, 'OutputView','full');
% figure(image_counter);
% image_counter = image_counter + 1;
% imagesc(RGBRect)
% set(gca,'dataAspectRatio',[1 1 1])
% title('grayfromRGB Rectified')
% 
% figure(image_counter);
% image_counter = image_counter + 1;
% imagesc(IRRect)
% set(gca,'dataAspectRatio',[1 1 1])
% title('IR Rectified')
% 
% figure(image_counter);
% image_counter = image_counter + 1;
% imagesc(stereoAnaglyph(RGBRect, IRRect));
% set(gca,'dataAspectRatio',[1 1 1])
% title('Rectified Frames');

%% Compute Disparity
% disparityMap = disparitySGM(RGBRect, IRRect);
% figure(image_counter);
% image_counter = image_counter + 1;
% imagesc(disparityMap, [0, 64]);
% title('Disparity Map');
% colormap jet
% colorbar

%% Reconstruct the 3-D Scene
% points3D = reconstructScene(disparityMap, stereoParams);
% figure(image_counter);
% image_counter = image_counter + 1;
% pcshow(points3D)
% title('Pointcloud')
% xlabel('X')
% ylabel('Y')
% zlabel('Z')


