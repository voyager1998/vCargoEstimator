%% Initialization, load RGB and IR images
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));
addpath(strcat(pwd,'/calibration'));
% RGB = imread(strcat(pwd, '/data/fix/fix80/RGBImage_1.png'));
% RGB = imread(strcat(pwd, '/data/data0618_1/RGBImage_7.png'));
% redChannel = RGB(:, :, 1);
% greenChannel = RGB(:, :, 2);
% blueChannel = RGB(:, :, 3);
% grayfromRGB = rgb2gray(RGB);

% load('calibration/panasonicRGBcameraParams.mat');
% C_rgb = rgbCameraParams.IntrinsicMatrix';

% rgb_undistort = undistortImage(grayfromRGB,rgbCameraParams);
% rgb_denoise= imbilatfilt(rgb_undistort, 1500, 5); % denoise for the Grayscale Img from RGB

% D = imread(strcat(pwd, '/data/calibration0725/boxA/DepthImage_35.png'));
D = imread(strcat(pwd, '/data/calibration0725/boxB/DepthImage_85.png'));
% D = imread(strcat(pwd, '/data/calibration0725/boxC/DepthImage_61.png'));
% D0 = imread(strcat(pwd, '/data/calibration0725/boxB/DepthImage_81.png'));
% D1 = imread(strcat(pwd, '/data/calibration0725/boxB/DepthImage_82.png'));
% D2 = imread(strcat(pwd, '/data/calibration0725/boxB/DepthImage_83.png'));
% IR = imread(strcat(pwd, '/data/calibration0725/boxB/Gray_81.png'));

% D = imread(strcat(pwd, '/data/fix/fix80/DepthImage_1.png')); 
% D = imread(strcat(pwd, '/data/fix/fix80/DepthImage_4.png'));
% D = imread(strcat(pwd, '/data/data0618_1/DepthImage_4.png'));

% D = imread(strcat(pwd, '/data/data0713/DepthImage_9.png'));
% D = imread(strcat(pwd, '/data/boxC_3/DepthImage_0.png'));
% D = double((D0 + D1 + D2)/3);
D = double(D);

D = D/16;

% D = medfilt2(D, [7 7]);

% 2 choice of intrinsic matrix; notice also need to change in Method3: xxx
% load('calibration/fixorigincalibration.mat');
% C_ir = cameraParams.IntrinsicMatrix';
% D_undistort = undistortImage(D,cameraParams);

load('calibration/panasonicIRcameraParams.mat');
C_ir = irCameraParams.IntrinsicMatrix';



% 2 ways of eliminating bias
% Method 1: linearly, notice this method applies directly to depth rather than depth undistorted
% load('bias.mat');
% bias=p; % bias transformation calculated from bias_cancellation.m 
% D=double(D);
% D=polyval(bias,D);
% D_undistort = undistortImage(D,irCameraParams); 

% Method 2: pixel-wise
D_undistort = undistortImage(D,irCameraParams);
% load('bias_pixel.mat');
% D_undistort = A.*D_undistort + B;

D_undistort = 0.955 * D_undistort;

D_denoise = imbilatfilt(D_undistort, 1500, 5);
%D_denoise = medfilt2(D_denoise, [7 7]);

% IR = undistortImage(IR,irCameraParams);
% IR_denoise = imbilatfilt(D_undistort, 1500, 5);

figure(image_counter);
image_counter = image_counter + 1;
imagesc(D_denoise)
set(gca,'dataAspectRatio',[1 1 1])
title('Depth Denoised')

% IR = imread(strcat(pwd, '/data/fix/fix80/GrayImage_1.png'));
% IR_undistort = undistortImage(IR,irCameraParams);
% IR_denoise = imbilatfilt(IR_undistort, 1500, 5);

% 2d pixel in tof camera's perspective -> 3d world points in tof camera's coordinate system
pc_ir = tof2pc_mat(D_denoise, C_ir);

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
    inlier_thres = 15;
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

% We need to make sure Plane1 is groud, Plane2 is top surface
if ( -plane_models(1,4)/plane_models(1,3) < -plane_models(2,4)/plane_models(2,3) ) 
    plane_models([1 2],:) = plane_models([2 1],:);
    plane_points([1 2]) = plane_points([2 1]);
end



% allplane_points = plane_points;
% allplane_models = plane_models;

%% Method1: Process D_denoise directly (should not use)
% edge_thres = 0.01;
% D_edge = edge(D_denoise, 'Canny', edge_thres);
% upper_edge = bwareafilt(D_edge,1); % take out the biggest object
% 
% figure(image_counter);
% image_counter = image_counter + 1;
% imshowpair(D_edge, upper_edge,'montage');
% title('Use depth image directly: before and after processing')

%% Method2: Turn the upper plane back to 2D -> then process the 2D image (before performing RANSAC line fitting)
% I = irCameraParams;
% 
% upper_pos = worldToImage(I,eye(3,3),zeros(3,1),plane_points{2}); % notice, here 2 represents the upper surface
% upper_pos = round(upper_pos);
% 
% upper_2D = zeros(size(D)); % take the 3D points of upper plane to 2D
% for i = 1:size(upper_pos, 1)
%     if abs(upper_pos(i,1))>480 || abs(upper_pos(i,2))>640 %qyy's abs(upper_pos(i,1))>480 || abs(upper_pos(i,1))>640
%         continue;
%     end
%     upper_2D(upper_pos(i,1), upper_pos(i,2)) = 1;
% end
% 
% upper_2D = logical(upper_2D); % change from double 0,1 to logical 0,1
% 
% upper_2D_post = imfill(upper_2D, 'holes'); % fill in the holes; may also try imerode(), imdilate()
% upper_2D_post = bwareaopen(upper_2D_post, 2000); % reject small objects; may try simply pick big objects
% 
% figure(image_counter);
% image_counter = image_counter + 1;
% imshowpair(upper_2D,upper_2D_post,'montage');
% title('2D Upper plane: before and after processing')
% 
% % plot, could comment out
% % figure(image_counter);
% % image_counter = image_counter + 1;
% % imshow(upper_2D);
% % set(gca,'dataAspectRatio',[1 1 1])
% % title('Upper plane')
% 
% edge_thres = 0.05;
% upper_edge = edge(upper_2D_post, 'Canny', edge_thres);
% 
% % edge_figure, should have
% figure(image_counter);
% edge_figure = image_counter;
% image_counter = image_counter + 1;
% imshow(upper_edge);
% set(gca,'dataAspectRatio',[1 1 1])
% title('Upper plane - edge')
% 
% % ------------------------------------------------------------
% % D_upperEdge, pc_upperEdge_ir are not used at this time
% % ------------------------------------------------------------
% % D_upperEdge = D_denoise .* double(upper_edge);
% % D_upperEdge(D_upperEdge == 0) = 2^12;
% 
% % plot, could comment out
% % figure(image_counter);
% % image_counter = image_counter + 1;
% % imagesc(D_upperEdge);
% % set(gca,'dataAspectRatio',[1 1 1])
% % title('Upper plane - depth')
% 
% % pc_upperEdge_ir = tof2pc(D_upperEdge, C_ir); 
% 
% % plot, could comment out
% % figure(image_counter);
% % image_counter = image_counter + 1;
% % pcshow(pc_upperEdge_ir)
% % title('Pointcloud - Upper Edge')
% % xlabel('X')
% % ylabel('Y')
% % zlabel('Z')
% % -------------------------------------------------------

%% Method 3: Most complicated - use the 2D plane to find ROI, then process the depth image
I = irCameraParams; % notice the 2 choice of intrinsic matrix
% I = cameraParams;

upper_pos = worldToImage(I,eye(3,3),zeros(3,1),plane_points{2}); % notice, here 2 represents the upper surface

upper_pos = round(upper_pos);

upper_2D = zeros(size(D)); % take the 3D points of upper plane to 2D
for i = 1:size(upper_pos, 1)
    if abs(upper_pos(i,1))>480 || abs(upper_pos(i,2))>640 %qyy's abs(upper_pos(i,1))>480 || abs(upper_pos(i,1))>640
        continue;
    end
    upper_2D(upper_pos(i,1), upper_pos(i,2)) = 1; % notice, previously upper_2D(upper_pos(i,2), upper_pos(i,1)) = 1;
end

upper_2D = logical(upper_2D); % change from double 0,1 to logical 0,1

upper_2D_post = imfill(upper_2D, 'holes'); % fill in the holes; may also try imerode(), imdilate()
%upper_2D_post = bwareaopen(upper_2D_post, 2000); % reject small objects; may try simply pick big objects
upper_2D_post = bwareafilt(upper_2D_post,1); % only keep the biggest object; aggressive

figure(image_counter);
image_counter = image_counter + 1;
imshowpair(upper_2D,upper_2D_post,'montage');
title('2D Upper plane: before and after processing')


% find the rectangle that contain the upper plane
stats = regionprops(upper_2D_post);
% show the image and draw the detected rectangles on it
figure(image_counter);
edge_figure = image_counter;
image_counter = image_counter + 1;
imagesc(upper_2D_post); 
title('Find the Region of Interest');
hold on;
for i = 1:numel(stats)
    rectangle('Position', stats(i).BoundingBox, ...
    'Linewidth', 1, 'EdgeColor', 'r');
end

enlarge_range = 15; % manually make the range larger

% notice BoundingBox =  [x y width height], but image is [col row], and col is reverse from y
col_min = max(1, round(stats(1).BoundingBox(2)) - enlarge_range);
col_max = min(480, round(stats(1).BoundingBox(2) + stats(1).BoundingBox(4)) + enlarge_range);
row_min = max(1, round(stats(1).BoundingBox(1)) - enlarge_range);
row_max = min(640, round(stats(1).BoundingBox(1) + stats(1).BoundingBox(3)) + enlarge_range);


D_smallPlane = D_denoise(col_min:col_max, row_min:row_max);


% col = [col_min col_max col_max col_min];
% row = [row_min row_min row_max row_max];

% D_smallPlane = imdilate(D_smallPlane,strel('disk',10));
% D_edge = bwmorph(D_smallPlane,'thin',inf);

edge_thres = 0.1;
D_smallEdge = edge(D_smallPlane, 'Canny', edge_thres); % edge detection on the small portion of image

D_edge = zeros(size(D_denoise));
D_edge(col_min:col_max, row_min:row_max) = D_smallEdge;

% upper_edge = imdilate(D_edge,strel('disk',5));
% upper_edge = bwmorph(upper_edge,'thin',inf);

upper_edge = bwareaopen(logical(D_edge), 100);
%upper_edge = bwareafilt(logical(upper_edge),1); % always take out the biggest, somewhat brute force


figure(image_counter);
image_counter = image_counter + 1;
imshowpair(D_edge,upper_edge,'montage');
title('Edge of depth image: before and after processing');


figure(image_counter);
image_counter = image_counter + 1;
imagesc(upper_edge);
title("Method3: Edge of Upper plane");



%% Then use ransac to fit lines (Input: edge image; Output: 4 lines)
numlines = 4;
edge_image = upper_edge; 

[rows,cols] = find(edge_image == true);
% edge_pts=[rows cols];
edge_pts=[cols rows];

line_2dmodels=zeros(numlines,2);
k=1;
% line_points{1,numlines}=[];
figure(image_counter);
image_counter = image_counter + 1;
imshow(edge_image);
title('Fit 4 lines');
hold on

for i=1:numlines
    sampleSize = 2; % number of points to sample per trial
    maxDistance = 20; % max allowable distance for inliers

    fitLineFcn = @(points) polyfit(points(:,2),points(:,1),1); % fit function using polyfit
    evalLineFcn = ...   % distance evaluation function
      @(model, points) sum((points(:, 1) - polyval(model, points(:,2))).^2,2);
%     evalLineFcn = ...   % distance evaluation function
%       @(model, points) sum(abs(points(:, 1) - polyval(model, points(:,2))),2)/sqrt(1+model(1,1));
    [modelRANSAC, inlierIdx] = ransac(edge_pts,fitLineFcn,evalLineFcn, ...
      sampleSize,maxDistance);
     
   if (abs(modelRANSAC(1)) < 3) % avoid slope being too steep
        edge_pts(inlierIdx==1,:)=[];    
        line_2dmodels(k,:)=modelRANSAC;   
        y = 0:480;
        x = modelRANSAC(1)*y+modelRANSAC(2);
        plot(x,y,'LineWidth',2)
        k=k+1;
    else
        fitLineFcn = @(points) polyfit(points(:,1),points(:,2),1); % fit function using polyfit
        evalLineFcn = ...   % distance evaluation function
          @(model, points) sum((points(:, 2) - polyval(model, points(:,1))).^2,2);
        [modelRANSAC, inlierIdx] = ransac(edge_pts,fitLineFcn,evalLineFcn, ...
          sampleSize,maxDistance);
        edge_pts(inlierIdx==1,:)=[];  
        line_2dmodels(k,1) = 1/modelRANSAC(1);
        line_2dmodels(k,2) = -modelRANSAC(2)/modelRANSAC(1);
        x = 0:640;
        y = modelRANSAC(1)*x+modelRANSAC(2);
        plot(x,y,'LineWidth',2)
        k=k+1;
    end
    

    
    
%     fitLineFcn = @(points) pca(points([2 1],:));
%     evalLineFcn = ...   % distance evaluation function
%       @(model, points) sum( abs(points(:,1) - model(2,1) - model(1,1) * points(:,2)))/sqrt(1+ model(2,1)^2)
%     [modelRANSAC, inlierIdx] = ransac(edge_pts,fitLineFcn,evalLineFcn, ...
%       sampleSize,maxDistance);
      
        
%     x = 0:640;
%     y = modelRANSAC(1)*x+modelRANSAC(2);
    
end
xlim([0 640]);
ylim([0 480]);
legend({},'Location','southwest');
hold off;



%% Method1: detect corner in depth image (2D)
% corner_2D = zeros(numlines,2);
% corner_count = 0;
% for i=1:(numlines-1)
%     for j = (i+1):numlines
%         a1 = line_2dmodels(i,1); b1 = line_2dmodels(i,2); % y = ax+b
%         a2 = line_2dmodels(j,1); b2 = line_2dmodels(j,2);
%         
%         x = (b2 - b1)/(a1 - a2); % solve a1*x + b1 = a2*x + b2
%         y = a1 * x + b1;
%         
%         if (x>0) && (x<480) && (y>0) && (y<640) 
%             corner_count = corner_count + 1;
%             corner_2D(corner_count, :) = [x y];
%         end            
%     end
% end
% 
% % plot(corner_2D(:,1), corner_2D(:,2), 'o');
% 
% corner_2D = round(corner_2D);
% D_corner2D = [corner_2D, zeros(size(corner_2D, 1), 1)];
% for i = 1:size(corner_2D, 1)
%     x = D_corner2D(i,1);
%     y = D_corner2D(i,2);
%     D_corner2D(i,3) = D_denoise(x, y);
% end
% 
% pc_corner = pixel2pc(D_corner2D, C_ir);
% corner_3D = proj2plane(pc_corner, plane_models(2,:)); % here, upper plane is 2, need to revise
% 
% figure(image_counter);
% image_counter = image_counter + 1;
% pcshow(plane_points{1}, [bitshift(bitand(1,4),-2) bitshift(bitand(1,2),-1) bitand(1,1)]);
% hold on;
% pcshow(plane_points{2}, [bitshift(bitand(2,4),-2) bitshift(bitand(2,2),-1) bitand(2,1)]);
% title('Pointcloud - Upper Plane vs. Corner (2D Method)')
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% 
% pcshow(pc_corner, [bitshift(bitand(5,4),-2) bitshift(bitand(5,2),-1) bitand(5,1)], 'MarkerSize',400);
% pcshow(corner_3D, [bitshift(bitand(6,4),-2) bitshift(bitand(6,2),-1) bitand(6,1)], 'MarkerSize',400);
% hold off;

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
        if (abs(dot(u1,u2))<0.5) && corner_count < numlines
            corner_count = corner_count + 1;            
            corner_3D(corner_count, :) = findIntersection(p1, pm_lines(i,:), pm_lines(j,:));
        end            
    end
end

% plane_points{2} = plane_points{2}( plane_points{2}(:,1) > min(corner_3D(:,1)) & ...
%     plane_points{2}(:,1) < max(corner_3D(:,1)) & ...
%     plane_points{2}(:,2) > min(corner_3D(:,2)) & ...
%     plane_points{2}(:,2) < max(corner_3D(:,2)), : );

figure(image_counter);
image_counter = image_counter + 1;
pcshow(plane_points{1}, [bitshift(bitand(1,4),-2) bitshift(bitand(1,2),-1) bitand(1,1)]);
hold on;
pcshow(plane_points{2}, [bitshift(bitand(2,4),-2) bitshift(bitand(2,2),-1) bitand(2,1)]);
title('Pointcloud - Upper Plane vs. Corner (3D method)')
xlabel('X')
ylabel('Y')
zlabel('Z')


% corner_3D = corner_3D(:,[2 1 3]);
pcshow(corner_3D, [bitshift(bitand(4,4),-2) bitshift(bitand(4,2),-1) bitand(4,1)], 'MarkerSize',400);
hold off;

%% calculate dimension
d = []; 

for i=1:3
    for j=(i+1):4
        dist = sqrt(sum(abs(corner_3D(i,:) - corner_3D(j,:)).^2));
        d = [d dist];
    end
end


l=(median(d(1:3))+median(d(4:6)))/2;
w=(min(d(1:3))+min(d(4:6)))/2;


% h = plane_dist(plane_models(1,:), plane_models(2,:), plane_points{1}, plane_points{2});
% h = Cal_height(plane_points, plane_models); % another way to calculate height, performance is similar

h = Cal_h(plane_models(1,:), plane_models(2,:), plane_points{1}, plane_points{2});

dimension=[h l w];
fprintf('Measured: Length = %.2f, Width = %.2f, Height = %.2f\n',dimension(2),dimension(3),dimension(1));
fprintf('Ground truth: L = 318 mm, W = 300 mm, H = 160 mm (box A)\n');
%fprintf('Ground truth: L = 288 mm, W = 188 mm, H = 115 mm (box C)\n');
vol=dimension(2)*dimension(3)*dimension(1);
fprintf('Vol=%.0f mm^3\n', vol);
trueV=160*318*300;
error=(vol-trueV)/trueV;
fprintf('Error=%.1f\n',error*100);