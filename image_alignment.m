%% Initialization, load two depth images
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));
addpath(strcat(pwd,'/calibration'));

D1 = imread(strcat(pwd, '/data/data0618_1/DepthImage_5.png'));
D2 = imread(strcat(pwd, '/data/data0618_1/DepthImage_8.png'));
D1 = D1/16;
D2 = D2/16;
load('calibration/ir.mat');
C = ircameraParams.IntrinsicMatrix';
D1 = undistortImage(D1,ircameraParams);
D2 = undistortImage(D2,ircameraParams);

figure(image_counter);
image_counter = image_counter + 1;
imshowpair(D1, D2,'Scaling','joint')
title('misaligned depth images');

%% Transform point cloud from the perpective of ToF camera to RGB camera


%% Align two depth images
% find transform martix from gray image
IR1 = imread(strcat(pwd, '/data/data0618_1/GrayImage_5.png'));
IR2 = imread(strcat(pwd, '/data/data0618_1/GrayImage_8.png'));

[optimizer, metric] = imregconfig('monomodal');
optimizer.MaximumIterations = 300;
% [IR2_registered] = imregister(IR2, IR1, 'affine', optimizer, metric);
tform = imregtform(IR2,IR1,'affine',optimizer,metric);
Rfixed = imref2d(size(IR1));

% apply the transform matrix on depth image
D2_registered = imwarp(D2,tform,'OutputView',Rfixed);

figure(image_counter);
image_counter = image_counter + 1;
imshowpair(D1, D2_registered,'Scaling','joint')
title('aligned depth images');

pc1 = tof2pc(D1, C);
pc2 = tof2pc(D2_registered, C);
figure(image_counter);
hold on;
image_counter = image_counter + 1;
pcshow(pc1, [0 1 0])
pcshow(pc2, [0 0 1])
title('original Pointcloud')
xlabel('X')
ylabel('Y')
zlabel('Z')

%% Register two point clouds using ICP algorithm
% https://www.mathworks.com/help/vision/examples/3-d-point-cloud-registration-and-stitching.html
% construct point cloud
pc1 = tof2pc(D1, C);
pc2 = tof2pc(D2, C);
pc1 = pointCloud(pc1);
pc2 = pointCloud(pc2);

% downsample
gridSize = 0.1;
fixed = pcdownsample(pc1,'gridAverage', gridSize);
moving = pcdownsample(pc2,'gridAverage', gridSize);

% align using ICP
tform = pcregistericp(moving,fixed,'Metric','pointToPlane','Extrapolate',true); % moving, fixed
pc2_reg = pctransform(pc2,tform);

% merge to construct a world scene
mergeSize = 0.015;
pcScene = pcmerge(pc1, pc2_reg, mergeSize);

%% Align RGB and Gray imgae
fixed = rgb2gray(imread(strcat(pwd, '/data/data0614/RGBImage_1.png')));
moving = imread(strcat(pwd, '/data/data0614/GrayImage_1.png'));

% scale intensity of ir and gray image
fixed = histeq(fixed);
figure(image_counter);
image_counter = image_counter + 1;
imshow(fixed);
title('Scaled rgb gray image')

moving = histeq(moving);
figure(image_counter);
image_counter = image_counter + 1;
imshow(moving);
title('Scaled gray image')

figure(image_counter);
image_counter = image_counter + 1;
imshowpair(fixed, moving,'Scaling','joint');
title('misaligned rgb and gray images');

[optimizer, metric] = imregconfig('multimodal');
optimizer.InitialRadius = optimizer.InitialRadius/3.5;
optimizer.MaximumIterations = 1000;
% movingRegistered = imregister(moving, fixed, 'affine', optimizer, metric);
tform = imregtform(moving,fixed,'affine',optimizer,metric);
Rfixed = imref2d(size(fixed));
moving_registered = imwarp(moving,tform,'OutputView',Rfixed);

figure(image_counter);
image_counter = image_counter + 1;
imshow(moving_registered);
title('registered depth images');

figure(image_counter);
image_counter = image_counter + 1;
imshowpair(fixed, moving_registered,'Scaling','joint');
title('aligned rgb and gray images');
