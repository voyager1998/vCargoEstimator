%% Initialization
clear; close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));


%% Load RGB image; Compute edges
RGB = imread(strcat(pwd, '/data/dataSmall/RGBImage_8.png'));

% turn RGB to gray
grayfromRGB = rgb2gray(RGB);

edge_gray = edge(grayfromRGB,'canny');

figure(image_counter);
image_counter = image_counter + 1;
imshow(edge_gray)
title('Edges in gray image')

%% Load IR image; Compute edges
IR = imread(strcat(pwd, '/data/dataSmall/GrayImage_8.png'));

edge_ir = edge(IR,'canny');

figure(image_counter);
image_counter = image_counter + 1;
imshow(edge_ir)
title('Edges in IR image')

%% Compute intersection of edges

