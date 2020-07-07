%% Initialization, load RGB and IR images
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));

RGB = imread(strcat(pwd, '/data/fix/fix80/RGBImage_1.png'));
% RGB = imread(strcat(pwd, '/data/data0618_1/RGBImage_7.png'));
redChannel = RGB(:, :, 1);
greenChannel = RGB(:, :, 2);
blueChannel = RGB(:, :, 3);
grayfromRGB = rgb2gray(RGB);
load('calibration/panasonicRGBcameraParams.mat');
C_rgb = rgbCameraParams.IntrinsicMatrix';
rgb_undistort = undistortImage(grayfromRGB,rgbCameraParams);
rgb_denoise= imbilatfilt(rgb_undistort, 1500, 5); % denoise for the Grayscale Img from RGB

D = imread(strcat(pwd, '/data/fix/fix80/DepthImage_1.png'));
% D = imread(strcat(pwd, '/data/data0618_1/DepthImage_7.png'));
D = D/16;

 load('calibration/panasonicIRcameraParams.mat');
% C_ir = irCameraParams.IntrinsicMatrix';
% D_undistort = undistortImage(D,irCameraParams);


load('calibration/fixorigincalibration.mat');
C_ir = cameraParams.IntrinsicMatrix';
D_undistort = undistortImage(D,cameraParams);
D_denoise = imbilatfilt(D_undistort, 1500, 5);


figure(image_counter);
image_counter = image_counter + 1;
imagesc(D_denoise)
set(gca,'dataAspectRatio',[1 1 1])
title('Depth Denoised')

IR = imread(strcat(pwd, '/data/fix/fix80/GrayImage_1.png'));
IR_undistort = undistortImage(IR,irCameraParams);
IR_denoise = imbilatfilt(IR_undistort, 1500, 5);

% 2d pixel in tof camera's perspective -> 3d world points in tof camera's coordinate system
pc_ir = tof2pc(D_denoise, C_ir);

%% RANSAC fit plane from tof's pc
pc = pc_ir;

numplanes = 2;
iterations = 100;
subset_size = 3;

ransac_figure = image_counter;
figure(ransac_figure);
hold on;
image_counter = image_counter + 1;
plane_models = zeros(numplanes,4);
plane_points{1,4} = [];
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
title('fit plane using pc from rgb');
xlabel('X');
ylabel('Y');
zlabel('Z');
hold off;

%% Turn the upper plane back to 2D -> then process the 2D image (before performing edge detection)
I = irCameraParams;

upper_pos = worldToImage(I,eye(3,3),zeros(3,1),plane_points{2}); % notice, here 2 represents the upper surface
upper_pos = round(upper_pos);

upper_2D = zeros(size(D)); % take the 3D points of upper plane to 2D
for i = 1:size(upper_pos, 1)
    upper_2D(upper_pos(i,2), upper_pos(i,1)) = 1;
end

upper_2D = logical(upper_2D); % change from double 0,1 to logical 0,1

upper_2D_post = imfill(upper_2D, 'holes'); % fill in the holes
upper_2D_post = bwareaopen(upper_2D_post, 2000); % reject small objects

figure(image_counter);
image_counter = image_counter + 1;
imshowpair(upper_2D,upper_2D_post,'montage');
title('2D Upper plane: before and after processing')

% plot, could comment out
% figure(image_counter);
% image_counter = image_counter + 1;
% imshow(upper_2D);
% set(gca,'dataAspectRatio',[1 1 1])
% title('Upper plane')

edge_thres = 0.05;
upper_edge = edge(upper_2D_post, 'Canny', edge_thres);

% edge_figure, should have
figure(image_counter);
edge_figure = image_counter;
image_counter = image_counter + 1;
imshow(upper_edge);
set(gca,'dataAspectRatio',[1 1 1])
title('Upper plane - edge')

% ------------------------------------------------------------
% D_upperEdge, pc_upperEdge_ir are not used at this time
% ------------------------------------------------------------
D_upperEdge = D_denoise .* uint16(upper_edge);
D_upperEdge(D_upperEdge == 0) = 2^12;

% plot, could comment out
% figure(image_counter);
% image_counter = image_counter + 1;
% imagesc(D_upperEdge);
% set(gca,'dataAspectRatio',[1 1 1])
% title('Upper plane - depth')

pc_upperEdge_ir = tof2pc(D_upperEdge, C_ir); 

% plot, could comment out
% figure(image_counter);
% image_counter = image_counter + 1;
% pcshow(pc_upperEdge_ir)
% title('Pointcloud - Upper Edge')
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% -------------------------------------------------------

%% use 2D edge detection -> then use ransac to fit lines
numlines = 4;
edge_image = upper_edge; 

[rows,cols] = find(edge_image == true);
ROI=(rows>100); % need to change later on, to soft-encode
edge_pts = [rows(ROI),cols(ROI)];
line_2dmodels=zeros(numlines,2);
k=1;
% line_points{1,numlines}=[];
figure(edge_figure);
hold on
for i=1:numlines
    sampleSize = 2; % number of points to sample per trial
    maxDistance = 100; % max allowable distance for inliers

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

%% Method1: detect corner in depth image (2D)
corner_2D = zeros(numlines,2);
corner_count = 0;
for i=1:(numlines-1)
    for j = (i+1):numlines
        a1 = line_2dmodels(i,1); b1 = line_2dmodels(i,2); % y = ax+b
        a2 = line_2dmodels(j,1); b2 = line_2dmodels(j,2);
        
        x = (b2 - b1)/(a1 - a2); % solve a1*x + b1 = a2*x + b2
        y = a1 * x + b1;
        
        if (x>0) && (x<640) && (y>0) && (y<480) 
            corner_count = corner_count + 1;
            corner_2D(corner_count, :) = [x y];
        end            
    end
end

% plot(corner_2D(:,1), corner_2D(:,2), 'o');

corner_2D = round(corner_2D);
D_corner2D = [corner_2D, zeros(size(corner_2D, 1), 1)];
for i = 1:size(corner_2D, 1)
    x = D_corner2D(i,1);
    y = D_corner2D(i,2);
    D_corner2D(i,3) = D_denoise(y, x);
end

pc_corner = pixel2pc(D_corner2D, C_ir);
pc_corner_proj = proj2plane(pc_corner, plane_models(2,:)); % here, upper plane is 2, need to revise

figure(image_counter);
image_counter = image_counter + 1;
pcshow(plane_points{1}, [bitshift(bitand(1,4),-2) bitshift(bitand(1,2),-1) bitand(1,1)]);
hold on;
pcshow(plane_points{2}, [bitshift(bitand(2,4),-2) bitshift(bitand(2,2),-1) bitand(2,1)]);
title('Pointcloud - Upper Plane vs. Corner (2D Method)')
xlabel('X')
ylabel('Y')
zlabel('Z')

pcshow(pc_corner, [bitshift(bitand(5,4),-2) bitshift(bitand(5,2),-1) bitand(5,1)], 'MarkerSize',400);
pcshow(pc_corner_proj, [bitshift(bitand(6,4),-2) bitshift(bitand(6,2),-1) bitand(6,1)], 'MarkerSize',400);
hold off;

d = []; 

for i=1:3
    for j=(i+1):4
        dist = sqrt(sum(abs(pc_corner_proj(i,:) - pc_corner_proj(j,:)).^2));
        d = [d dist];
    end
end
    
d % the distance is wrong because not in the same plane

%% Method2: detect corner in point cloud (3D)

p1=plane_models(2,:); % top plane
pm_lines = zeros(numlines, 4); % plane model of lines

for i=1:numlines % assume 2 lines are of interest
    pm_lines(i,:) = ( line2dTplane(line_2dmodels(i,:), C_ir) )';
end

corner_3D = zeros(numlines,3);
corner_count = 0;
for i=1:(numlines-1)
    for j = (i+1):numlines   
        % p1 is the top plane
        u1 = pm_lines(i,1:3)/ norm(pm_lines(i,1:3));
        u2 = pm_lines(j,1:3)/ norm(pm_lines(j,1:3));        
        if (abs(dot(u1,u2))<0.5) && corner_count <= numlines
            corner_count = corner_count + 1;            
            corner_3D(corner_count, :) = findIntersection(p1, pm_lines(i,:), pm_lines(j,:));
        end            
    end
end

figure(image_counter);
image_counter = image_counter + 1;
pcshow(plane_points{1}, [bitshift(bitand(1,4),-2) bitshift(bitand(1,2),-1) bitand(1,1)]);
hold on;
pcshow(plane_points{2}, [bitshift(bitand(2,4),-2) bitshift(bitand(2,2),-1) bitand(2,1)]);
title('Pointcloud - Upper Plane vs. Corner (3D method)')
xlabel('X')
ylabel('Y')
zlabel('Z')

pcshow(corner_3D, [bitshift(bitand(4,4),-2) bitshift(bitand(4,2),-1) bitand(4,1)], 'MarkerSize',400);
hold off;

d = []; 

for i=1:3
    for j=(i+1):4
        dist = sqrt(sum(abs(corner_3D(i,:) - corner_3D(j,:)).^2));
        d = [d dist];
    end
end
    
d % the distance is wrong because not in the same plane
