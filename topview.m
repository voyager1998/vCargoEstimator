%% Initialization, load depth images
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));

% D must be a top view
D = imread(strcat(pwd, '/data/fix/fix80/DepthImage_1.png'));
load('calibration/panasonicIRcameraParams.mat');
C_ir = irCameraParams.IntrinsicMatrix';

% eliminate bias
load('bias_pixelwise.mat'); % bias transformation calculated from bias_cancellation.m 
D=double(D);
D = D/16;
D_undistort = undistortImage(D,irCameraParams);
D_undistort = A.*D_undistort+B;
D_denoise = imbilatfilt(D_undistort, 1500, 5);

pc_ir = tof2pc_mat(D_denoise, C_ir);

%% RANSAC fit plane from tof's pc
pc = pc_ir;

numplanes = 4; % fit 2 planes: top plane and ground
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
% edge_thres = 0.01;
% 
% I = irCameraParams.Intrinsics;
% % find region of interest
% upper_pos = worldToImage(I,eye(3,3),zeros(3,1),plane_points{2}); % notice, here 2 represents the upper surface
% upper_pos = round(upper_pos);
% upper_2D = zeros(size(D)); % take the 3D points of upper plane to 2D
% for i = 1:size(upper_pos, 1)
%     if abs(upper_pos(i,1))>480 || abs(upper_pos(i,2))>640
%         continue;
%     end
%     upper_2D(upper_pos(i,1), upper_pos(i,2)) = 1;
% end
% upper_2D = logical(upper_2D); % change from double 0,1 to logical 0,1
% upper_2D_post = imfill(upper_2D, 'holes'); % fill in the holes
% upper_2D_post = bwareaopen(upper_2D_post, 5000); % reject small objects
% % find the rectangle that contain the upper plane
% stats = regionprops(upper_2D_post);
% enlarge_range = 15; % manually make the range larger
% % notice BoundingBox =  [x y width height], but image is [col row], and col is reverse from y
% row=stats.BoundingBox(1);
% col=stats.BoundingBox(2);
% w=stats.BoundingBox(3);
% h=stats.BoundingBox(4);
% col_min = round(col) - enlarge_range;
% col_max = round(col + h) + enlarge_range;
% row_min = round(row) - enlarge_range;
% row_max = round(row + w) + enlarge_range;
% if col_min<1
%     col_min=1;
% end
% if col_max>480
%     col_max=480;
% end
% if row_min<1
%     row_min=1;
% end
% if row_max>640
%     row_max=640;
% end
% 
% D_smallPlane = D_denoise(col_min:col_max,row_min:row_max);
% D_smallEdge = edge(D_smallPlane, 'Canny', edge_thres); % edge detection on the small portion of image
% 
% D_edge = zeros(size(D_denoise));
% D_edge(col_min:col_max, row_min:row_max) = D_smallEdge;
% 
% upper_edge = bwareafilt(logical(D_edge),1); % always take out the biggest, somewhat brute force
% edge_figure=image_counter;
% figure(edge_figure);
% image_counter = image_counter + 1;
% imshow(upper_edge);
% title("Edge of Upper plane");

% Method1: directly fit edge on upper plane
edge_thres = 0.1;
% find region of interest
upper_pos = worldToImage(irCameraParams,eye(3,3),zeros(3,1),plane_points{top_plane}); % notice, here 2 represents the upper surface
upper_pos = round(upper_pos);
upper_2D = zeros(size(D)); % take the 3D points of upper plane to 2D
for i = 1:size(upper_pos, 1)
    if abs(upper_pos(i,1))>480 || abs(upper_pos(i,2))>640
        continue;
    end
    upper_2D(upper_pos(i,1), upper_pos(i,2)) = 1;
end
upper_2D = logical(upper_2D); % change from double 0,1 to logical 0,1
upper_2D = imfill(upper_2D, 'holes'); % fill in the holes
upper_2D = bwareaopen(upper_2D, 5000); % reject small objects

% detect edge on 2d top plane
upper_edge = edge(upper_2D, 'Canny', edge_thres); % edge detection on the small portion of image

edge_figure=image_counter;
figure(edge_figure);
image_counter = image_counter + 1;
imshow(upper_edge);
title("Edge of Upper plane");

%% RANSAC fit edge in depth image
numlines = 4; % 4 edges of a rectangle

[rows,cols]=find(upper_edge==true);
edge_pts=[cols rows];
line_2dmodels=zeros(numlines,2);
k=1;
figure(edge_figure);
hold on
for i=1:numlines
    sampleSize = 2; % number of points to sample per trial
    maxDistance = 30; % max allowable distance for inliers

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
p1=plane_models(top_plane,:); % top plane
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
h = plane_dist(plane_models(1,:), plane_models(2,:), plane_points{1}, plane_points{2});

corners=zeros(numlines.*(numlines-1),3);
c=1;

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
            c=c+1;
        end
    end
end
c=c-1;
distances=zeros(6,1);
k=1;
for i=1:c-1
    for j=i+1:c
        distances(k)=norm(corners(i,:)-corners(j,:));
        k=k+1;
    end
end
l=(median(distances(1:3))+median(distances(4:6)))/2;
w=(min(distances(1:3))+min(distances(4:6)))/2;
dimension=[h l w];
fprintf('Length=%.0f, Width=%.0f, Height=%.0f\n',dimension(2),dimension(3),dimension(1));
vol=dimension(2)*dimension(3)*dimension(1);
fprintf('Vol=%.0f mm^3\n', vol);
trueV=160*318*300;
error=(vol-trueV)/trueV;
fprintf('Error=%.1f\n',error*100);

