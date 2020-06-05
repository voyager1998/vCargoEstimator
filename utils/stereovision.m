% Stereo Vision

%% Initialization, load stereo images
clear; close all;
image_counter = 1;
addpath(pwd);

load('handshakeStereoParams.mat');
videoFileLeft = 'handshake_left.avi';
videoFileRight = 'handshake_right.avi';