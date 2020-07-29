%% Initialization, load depth images
% clear; close all;
% image_counter = 1;
% addpath(pwd);
% addpath(strcat(pwd,'/utils'));

% D can be a top view, can be a side view
% top view
% D=imread(strcat(pwd, '/data/fix/fix90/DepthImage_0.png'));
% side view
% D=imread(strcat(pwd,'/data/data0618_1/DepthImage_0.png'));


function [dimension,img_counter]=calculate_dimension(image_counter,D,bias_method)
addpath(pwd);
addpath(strcat(pwd,'/utils'));

irCameraParams=load('calibration/panasonicIRcameraParams.mat').irCameraParams;
C_ir = irCameraParams.IntrinsicMatrix';
D = double(D);
D = D/16;
D = undistortImage(D,irCameraParams);
% eliminate bias
if bias_method==1 % Method1: pixel wise bias
    bias=load('bias_pixelwise.mat');
    D = bias.A.*D + bias.B;
elseif bias_method==2 % Method2: linear bias
    bias=load('bias_linear.mat').p;
    D=polyval(bias,D);
end
D_denoise = imbilatfilt(D, 1500, 5);

pc_ir = tof2pc_mat(D_denoise, C_ir);

%% RANSAC fit plane from tof's pc
numplanes = 2; % topview fit 2 planes

iterations = 100;
subset_size = 3;

pc = pc_ir;
ransac_figure = image_counter;
image_counter = image_counter + 1;
figure(ransac_figure);
hold on
plane_models = zeros(numplanes,4);
plane_points{1,numplanes} = [];
top_plane=1;
for i = 1:numplanes
    inlier_thres = 10;
    if (i == 1) 
        inlier_thres = 30;
    end
    noise_ths = ones(1, length(pc)) * inlier_thres;
    [plane_models(i,:), outlier_ratio, plane_area, inliers, best_inliers] ...
        = ransac_fitplane(pc, 1:length(pc), noise_ths, iterations, subset_size);
    % find top plane
    if i>1 && top_plane==1 % i=1 often is ground
        n1=plane_models(1,1:3);
        n2=plane_models(i,1:3);
        if abs((n1*n2')/(norm(n1).*norm(n2)))>0.5 % parallel planes
            top_plane=i;
        end
    end
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

%% Find edges - method1: directly fit edge on upper plane
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
edge_pts=[cols,rows];
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
% h = plane_dist(plane_models(1,:), plane_models(top_plane,:), plane_points{1}, plane_points{top_plane});
h = Cal_h(plane_points{1}, plane_points{top_plane}, plane_models(1,:), plane_models(top_plane,:));

%% project top-plane points on edge -  - Method2 using projection
figure(ransac_figure);
hold on
sample_points=500;
distances=zeros(4,1);
for j=1:4
    I = edge_3dmodels(j,1:3); % point on line
    u = edge_3dmodels(j,4:6); % direction vector of line

    total_points = size(plane_points{top_plane});
    projections=zeros(sample_points,3);
    dists=zeros(sample_points,1);
    k=1;
    for i=1:floor(total_points/sample_points):total_points
        P = plane_points{2}(i,:); % one point on the top plane
        v = P-I;
        projection = dot(u,v);
        Q = I+projection*u; % projected point of P on the line
        projections(k,:)=Q;
        dists(k)=projection;
        k=k+1;
        % plot3(P(1),P(2),P(3),'r.','MarkerSize',50)
        plot3(Q(1),Q(2),Q(3),'r.','MarkerSize',10)
    end
    syms t % for plotting line
    t=-500:500;
    line = I'+t.*(u'/norm(u'));
    plot3(line(1,:),line(2,:),line(3,:),'.','Color','white')
    
    num=floor(sample_points/(max(dists)-min(dists)));
    dists=sort(dists);
    idx_min=1;
%     while (dists(idx_min+3)-dists(idx_min) > 1 || dists(idx_min+4)-dists(idx_min+1) > 1 || dists(idx_min+5)-dists(idx_min+2) > 1)
    while (dists(idx_min+num)-dists(idx_min)>1)
        idx_min=idx_min+1;
    end
    idx_max=size(dists,1);
%     while (dists(idx_max)-dists(idx_max-3) > 1 || dists(idx_max-1)-dists(idx_max-4) > 1 || dists(idx_max-2)-dists(idx_max-5) > 1)
    while (dists(idx_max)-dists(idx_max-num)>1)
        idx_max=idx_max-1;
    end
    dist=dists(idx_max)-dists(idx_min);
    distances(j)=dist;
    fprintf('dist%d=%.1f\n',j,dist);
end
distances=sort(distances);
w=(distances(1)+distances(2))/2;
l=(distances(3)+distances(4))/2;
dimension=[h l w];
img_counter=image_counter;
end
