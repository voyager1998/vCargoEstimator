% Input: a tof depth image
% Output: possible planes in the image

%% Initialization, load depth image
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));
addpath(strcat(pwd,'/calibration'));

% initial dataset (that works)
%D = imread(strcat(pwd, '/data/0605/DepthImage_3.png')); % has tried 0(good), 3(bad)
%D = imread(strcat(pwd, '/data/data1/DepthImage_2.png')); % the picture is over-exposed

% a flat dataset, taken on the table of lab
%D = imread(strcat(pwd, '/data/dataBig/DepthImage_0.png'));
%RGB = imread(strcat(pwd, '/data/dataBig/RGBImage_0.png'));

% a good, "ground-truth" dataset
D = imread(strcat(pwd, '/data/data0614/DepthImage_0.png'));
IR = imread(strcat(pwd, '/data/data0614/GrayImage_0.png'));
RGB = imread(strcat(pwd, '/data/data0614/RGBImage_0.png'));

D = D/16; % depth image format is 16bit, but only 12bit is useful -> D/(2^4)

load('calibration/fixorigincalibration.mat');
C = cameraParams.IntrinsicMatrix'; % intrinsic matrix
D_undistort = undistortImage(D,cameraParams); % a matlab built-in function,should we use it?, or, is it useless?

%D_undistort = D;

figure(image_counter); % a counter of numbers
image_counter = image_counter + 1;
imagesc(D)
set(gca,'dataAspectRatio',[1 1 1])
title('Depth')

figure(image_counter);
image_counter = image_counter + 1;
imagesc(D_undistort)
set(gca,'dataAspectRatio',[1 1 1])
title('Depth Undistorted')

figure(image_counter);
image_counter = image_counter + 1;
IR_undistort = undistortImage(IR,cameraParams);
imagesc(IR_undistort)
set(gca,'dataAspectRatio',[1 1 1])
title('IR image')

figure(image_counter);
image_counter = image_counter + 1;
imagesc(RGB)
set(gca,'dataAspectRatio',[1 1 1])
title('RGB image')

%% Denosing - to the depth image: two paramters to specify, mu_reg and c_thres_rate
addpath(strcat(pwd,'/denoising'));
[imgHeight, imgWidth] = size(D_undistort);

D_resize = [D_undistort; zeros(imgWidth-imgHeight, imgWidth)]; %make it square, in order to use SB_ATV
D_unrolled = D_resize(:);
D_unrolled = double(D_unrolled);

mu_reg = 60; %   mu: regularisation parameter

D_unrolled_denoise_atv = SB_ATV(D_unrolled, mu_reg); % use ATV, becuase ITV cannot run 

fprintf('ATV Rel.Err = %g\n',norm(D_unrolled_denoise_atv(:) - D_unrolled(:)) ...
    / norm(D_unrolled_denoise_atv(:)));

D_denoise_atv = reshape(D_unrolled_denoise_atv, imgWidth, imgWidth);
D_denoise_atv = D_denoise_atv(1:imgHeight, 1:imgWidth);

figure(image_counter);
image_counter = image_counter + 1;
imagesc(D_denoise_atv);
set(gca,'dataAspectRatio',[1 1 1])
title('Depth Denoised (standard)')

% Denoising using adaptive way
IR_resize = [IR_undistort; zeros(imgWidth-imgHeight, imgWidth)];
IR_unrolled = IR_resize(:);
IR_unrolled = double(IR_unrolled);

c_thres_rate = 0.005 % contolls the threshold c
c = c_thres_rate * (max(IR_unrolled))^2;

weight_fidelity = 1/c * min(c, IR_unrolled.^2); % check the formula in tof.pdf
%weight_fidelity = ones(length(IR_unrolled),1);

D_unrolled_denoise_atv_adaptive = SB_ATV_weighted(D_unrolled, mu_reg, weight_fidelity);
fprintf('ATV Rel.Err = %g\n',norm(D_unrolled_denoise_atv_adaptive(:) - D_unrolled(:)) ...
    / norm(D_unrolled_denoise_atv_adaptive(:)));

D_denoise_atv_adaptive = reshape(D_unrolled_denoise_atv_adaptive, imgWidth, imgWidth);
D_denoise_atv_adaptive = D_denoise_atv_adaptive(1:imgHeight, 1:imgWidth);

figure(image_counter);
image_counter = image_counter + 1;
imagesc(D_denoise_atv_adaptive);
set(gca,'dataAspectRatio',[1 1 1])
title('Depth Denoised (adaptive£©')

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

%% Construct Point Cloud - for denoised (standard)
pc_denoised = tof2pc(D_denoise_atv, C);
figure(image_counter);
image_counter = image_counter + 1;
pcshow(pc_denoised)
title('Pointcloud Denoised (standard)')
xlabel('X')
ylabel('Y')
zlabel('Z')


%% Construct Point Cloud - for denoised (adaptive)
pc_denoised_adaptive = tof2pc(D_denoise_atv_adaptive, C);
figure(image_counter);
image_counter = image_counter + 1;
pcshow(pc_denoised_adaptive)
title('Pointcloud Denoised (adaptive)')
xlabel('X')
ylabel('Y')
zlabel('Z')

%% RANSAC: need to specify noise_thres
pc = originpc;

iterations = 100;
subset_size = 3; %number of points to specify a plane

figure(image_counter);
hold on;
image_counter = image_counter + 1;
numplanes = 4; %number of planes to fit
plane_models = zeros(numplanes,4);
noise_thres = 20; %noise threshold
for i = 1:numplanes %fit the plane, than take out the plane
    noise_ths = ones(1, length(pc)) * noise_thres;
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


%% RANSAC - denoised (standard)
%pc = originpc;
pc = pc_denoised;

iterations = 100;
subset_size = 3; %number of points to specify a plane

figure(image_counter);
hold on;
image_counter = image_counter + 1;
numplanes = 4; %number of planes to fit
plane_models = zeros(numplanes,4);
noise_thres = 20; %noise threshold
for i = 1:numplanes %fit the plane, than take out the plane
    noise_ths = ones(1, length(pc)) * noise_thres;
    [plane_models(i,:), outlier_ratio, plane_area, inliers, best_inliers] ...
        = ransac_fitplane(pc, 1:length(pc), noise_ths, iterations, subset_size);
    pc(best_inliers, :) = [];
    plane_points{i} = inliers;
    pcshow(inliers, [bitshift(bitand(i,4),-2) bitshift(bitand(i,2),-1) bitand(i,1)]);
end
% plane1 blue, plane2 green, plane3 cyan, plane4 red
title('inliers - denoised (standard)')
xlabel('X');
ylabel('Y');
zlabel('Z');

%% RANSAC - denoised (adaptive)
%pc = originpc;
pc = pc_denoised_adaptive;

iterations = 100;
subset_size = 3; %number of points to specify a plane

figure(image_counter);
hold on;
image_counter = image_counter + 1;
numplanes = 4; %number of planes to fit
plane_models = zeros(numplanes,4);
noise_thres = 20; %noise threshold
for i = 1:numplanes %fit the plane, than take out the plane
    noise_ths = ones(1, length(pc)) * noise_thres;
    [plane_models(i,:), outlier_ratio, plane_area, inliers, best_inliers] ...
        = ransac_fitplane(pc, 1:length(pc), noise_ths, iterations, subset_size);
    pc(best_inliers, :) = [];
    plane_points{i} = inliers;
    pcshow(inliers, [bitshift(bitand(i,4),-2) bitshift(bitand(i,2),-1) bitand(i,1)]);
end
% plane1 blue, plane2 green, plane3 cyan, plane4 red
title('inliers - denoised (adaptive)')
xlabel('X');
ylabel('Y');
zlabel('Z');

%% calculate intersection line (not needed)
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

%% find intersection line
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
pcshow(plane_points{1}); % always the ground
pcshow(plane_points{2}); % need to specify by hand
title('parallel planes for height calculation');
height = plane_dist(plane_models(1,:), plane_models(2,:), plane_points{1}, plane_points{2});
% height = 340.7499 / 341.3144

%% calculate width and length
figure(image_counter);
image_counter = image_counter + 1;
pcshow(plane_points{3}); % also, need to specify by hand