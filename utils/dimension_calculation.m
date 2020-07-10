function dimension = dimension_calculation(D)
addpath(strcat(pwd,'/utils'));
load('calibration/panasonicIRcameraParams.mat','irCameraParams');
C_ir = irCameraParams.IntrinsicMatrix';
I = irCameraParams.Intrinsics;
D = D/16;
D = undistortImage(D,irCameraParams);

%% eliminate bias 
% if no bias.mat, run bias_cancellation.m first
% bias=load('bias.mat').p;
% D = double(D);
% D = polyval(bias,D);
D_denoise = imbilatfilt(D, 1500, 5);
pc_ir = tof2pc(D_denoise, C_ir);

%% RANSAC fit plane from tof's pc
numplanes = 2; % fit 2 planes: top plane and ground
iterations = 100;
subset_size = 3;

pc = pc_ir;
plane_models = zeros(numplanes,4);
plane_points{1,numplanes} = [];
for i = 1:numplanes
    inlier_thres = 10;
    if (i == 1) 
        inlier_thres = 30;
    end
    noise_ths = ones(1, length(pc)) * inlier_thres;
    [plane_models(i,:), ~, ~, inliers, best_inliers] ...
        = ransac_fitplane(pc, 1:length(pc), noise_ths, iterations, subset_size);
    pc(best_inliers, :) = [];
    plane_points{i} = inliers;
end

%% find edge from depth image
upper_pts = worldToImage(I,eye(3,3),zeros(3,1),plane_points{2}); % notice, here 2 represents the upper surface
edge_image=detect_ROIedge(upper_pts,D_denoise);
imshow(edge_image)
hold on;

%% RANSAC fit edge in depth image
numlines = 4; % 4 edges of a rectangle

[rows,cols]=find(edge_image==true);
edge_pts=[rows,cols];
line_2dmodels=zeros(numlines,2);
k=1;
for i=1:numlines
    sampleSize = 2; % number of points to sample per trial
    maxDistance = 150; % max allowable distance for inliers

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

%% 2d edge to 3d line
edge_3dmodels=zeros(numlines,6); % (1:3)=I,point on line % (4:6)=u,direction vector of line

p1=plane_models(2,:); % top plane
n1=p1(1:3);
M1=[0,0,-p1(4)./p1(3)];

for i=1:numlines % assume 2 lines are of interest
    pm = line2dTplane(line_2dmodels(i,:), C_ir);
    n2=pm(1:3);
    M2=[0,0,-pm(4)./pm(3)];
    
    [I, u, rc] = planes_intersection(n1, M1, n2', M2, 1);
    edge_3dmodels(i,:)=[I u]; % I point on line, u direction vector of line
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
end