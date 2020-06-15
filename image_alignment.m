%% Initialization, load RGB and IR images
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));

%% Align two depth images
fixed = imread(strcat(pwd, '/data/dataSmall/depthImage_1.png'));
moving = imread(strcat(pwd, '/data/dataSmall/DepthImage_2.png'));
figure(image_counter);
image_counter = image_counter + 1;
imshowpair(fixed, moving,'Scaling','joint')
title('misaligned depth images');

[optimizer, metric] = imregconfig('monomodal');
movingRegistered = imregister(moving, fixed, 'affine', optimizer, metric);
figure(image_counter);
image_counter = image_counter + 1;
imshowpair(fixed, movingRegistered,'Scaling','joint')
title('aligned depth images');
