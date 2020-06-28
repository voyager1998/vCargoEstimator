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

ransac_figure = image_counter;
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
        plot3(line(1,:),line(2,:),line(3,:),'-');
    end
end
% hold off;

%% calculate intersection point
% syms x y z
% v=[x;y;z;1];
% ekv1=plane_models(1,:)*v==0;
% ekv2=plane_models(2,:)*v==0;
% ekv3=plane_models(3,:)*v==0;
% xyz = solve([ekv1, ekv2, ekv3]);

%% calculate height
% figure(image_counter);
% hold on;
% image_counter = image_counter + 1;
% pcshow(plane_points{1});
% pcshow(plane_points{2});
% title('parallel planes for height calculation');
height = plane_dist(plane_models(1,:), plane_models(2,:), plane_points{1}, plane_points{2});
% height = 340.7499 / 341.3144

%% Load Depth image; Compute edges
edge_thres = 0.1;
edge_D = edge(D_undistort,'Canny', edge_thres);

figure(image_counter);
image_counter = image_counter + 1;
imagesc(edge_D)
title('Edges in Depth image')
hold on

%% RANSAC edge function based on Depth image
[rows,cols] = find(edge_D == true);
edge_pts = [rows,cols];

sampleSize = 2; % number of points to sample per trial
maxDistance = 2; % max allowable distance for inliers

fitLineFcn = @(points) polyfit(points(:,2),points(:,1),1); % fit function using polyfit
evalLineFcn = ...   % distance evaluation function
  @(model, points) sum((points(:, 1) - polyval(model, points(:,2))).^2,2);

[modelRANSAC, inlierIdx] = ransac(edge_pts,fitLineFcn,evalLineFcn, ...
  sampleSize,maxDistance);
x = 0:640;
y = modelRANSAC(1)*x+modelRANSAC(2);
plot(x,y)

%% From edge to plane; Compute plane intersection
figure(ransac_figure);
pm = line2dTplane(modelRANSAC, C);

% syms X Y Z1
% scale1=1./pm(3); % scale factor before z to 1
% pm=scale1*pm;
% 
% [X,Y]=meshgrid(-400:50:400);
% Z1=-pm(1).*X-pm(2).*Y-pm(4);
% surf(X,Y,Z1);

p1=plane_models(2,:);
n1=p1(1:3);
M1=[0,0,-p1(4)./p1(3)];

n2=pm(1:3);
M2=[0,0,-pm(4)./pm(3)];
[I, u, rc] = planes_intersection(n1, M1, n2', M2, 1);
edges(k,:)=[I u]; % I point on line, u direction vector of line
k=k+1;
syms t
t=-500:500;
line = I'+t.*(u'/norm(u'));
plot3(line(1,:),line(2,:),line(3,:),'o');

%% calculate width
% find parallel lines to the edge
parallel_lines=zeros(1,2);
m=1;
v1=edges(k-1,4:6);
v1=v1./norm(v1);
for i=1:k-2
    v2=edges(i,4:6);
    v2=v2./norm(v2);
    cos_val=dot(v1,v2);
    if abs(sin(acos(cos_val)))<0.5
        parallel_lines(m)=i;
        m=m+1;
    end
end

% calculate distance between the parallel lines
figure(ransac_figure);
syms t y1 y2
y1=I'+t.*(u'/norm(u'));
i=1;
I2=edges(i,1:3);
u2=edges(i,4:6);
y2=I2'+t.*(u2'/norm(u2'));
t=-500:500;
plot3(y2(1,:),y2(2,:),y2(3,:),'o');
% corner=solve(norm(y1-y2)<100,t);
% width = line_dist(u1,I1,u2,I2,points1,points2);

