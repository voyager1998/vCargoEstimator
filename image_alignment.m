%% Initialization, load two depth images
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));
addpath(strcat(pwd,'/calibration'));

D1 = imread(strcat(pwd, '/data/0605/DepthImage_3.png'));
D2 = imread(strcat(pwd, '/data/0605/DepthImage_1.png'));
D1 = D1/16;
D2 = D2/16;
load('calibration/ir.mat');
C = ircameraParams.IntrinsicMatrix';
D1 = undistortImage(D1,ircameraParams);
D2 = undistortImage(D2,ircameraParams);

figure(image_counter);
image_counter = image_counter + 1;
imagesc(D1)
set(gca,'dataAspectRatio',[1 1 1])
title('origiunal depth images');

figure(image_counter);
image_counter = image_counter + 1;
imshowpair(D1, D2,'Scaling','joint')
title('misaligned depth images');

%% Align two depth images
% find transform martix from gray image
IR1 = imread(strcat(pwd, '/data/0605/GrayImage_3.png'));
IR2 = imread(strcat(pwd, '/data/0605/GrayImage_1.png'));

[optimizer, metric] = imregconfig('monomodal');
optimizer.MaximumIterations = 300;
% [IR2_registered] = imregister(IR2, IR1, 'affine', optimizer, metric);
tform = imregtform(IR2,IR1,'affine',optimizer,metric);
Rfixed = imref2d(size(IR1));
% figure(image_counter);
% image_counter = image_counter + 1;
% imshowpair(IR1, IR2_registered,'Scaling','joint')
% title('aligned depth images');

% figure(image_counter);
% image_counter = image_counter + 1;
% imagesc(IR1)
% set(gca,'dataAspectRatio',[1 1 1])
% title('fixed IR image');
% 
% figure(image_counter);
% image_counter = image_counter + 1;
% imagesc(IR2_registered)
% set(gca,'dataAspectRatio',[1 1 1])
% title('registered IR image');

% apply the transform matrix on depth image
D2_registered = imwarp(D2,tform,'OutputView',Rfixed);

figure(image_counter);
image_counter = image_counter + 1;
imshowpair(D1, D2_registered,'Scaling','joint')
title('aligned depth images');

figure(image_counter);
image_counter = image_counter + 1;
imagesc(D2_registered)
set(gca,'dataAspectRatio',[1 1 1])
title('registered depth images');

%intersection=intersect(D1,D2_registered);
% s = size(D1);
% w = s(1);
% h = s(2);
% intersection=zeros(w,h);
% for i = 1: w
%     for j = 1: h
%         intersection(i,j)=0.5.*(D1(i,j)+D2_registered(i,j));
%     end
% end
% figure(image_counter);
% image_counter = image_counter + 1;
% imagesc(intersection)
% set(gca,'dataAspectRatio',[1 1 1])
% title('registered depth images');

pc_original = tof2pc(D1, C);
figure(image_counter);
image_counter = image_counter + 1;
pcshow(pc_original)
title('original Pointcloud')
xlabel('X')
ylabel('Y')
zlabel('Z')

pc = tof2pc(intersection, C);
figure(image_counter);
image_counter = image_counter + 1;
pcshow(pc)
title('Pointcloud')
xlabel('X')
ylabel('Y')
zlabel('Z')

% pc1 = tof2pc(D1, C);
% pc2 = tof2pc(D2_registered, C);
% pc = [pc1;pc2]; % union of two depth images
% 
% figure(image_counter);
% image_counter = image_counter + 1;
% pcshow(pc);
% title('union Pointcloud')
% xlabel('X')
% ylabel('Y')
% zlabel('Z')

%% Register two point clouds using ICP algorithm
%Construct Point Cloud
pc1 = tof2pc(D1, C);
pc2 = tof2pc(D2, C);
pc1 = pointCloud(pc1);
pc2 = pointCloud(pc2);

figure(image_counter);
image_counter = image_counter + 1;
pcshow(pc1);
title('fixed Pointcloud')
xlabel('X')
ylabel('Y')
zlabel('Z')


figure(image_counter);
image_counter = image_counter + 1;
pcshowpair(pc1, pc2);
title('Pointcloud')
xlabel('X')
ylabel('Y')
zlabel('Z')

[tform,pc2_reg] = pcregistericp(pc2,pc1); % moving, fixed
figure(image_counter);
image_counter = image_counter + 1;
pcshowpair(pc1, pc2_reg);
title('Aligned Pointcloud')
xlabel('X')
ylabel('Y')
zlabel('Z')

pc = intersect(pc1,pc2_reg,'rows');

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
