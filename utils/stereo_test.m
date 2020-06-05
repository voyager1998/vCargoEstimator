% Stereo Vision
%% Initialization
clear; close all;
image_counter = 1;
addpath(pwd);

%% get stereoParams from single cameraParams (not runnable)
load('calibration/rgb.mat');
RGBParams = cameraParams;
figure(image_counter);
image_counter = image_counter + 1;
showExtrinsics(RGBParams);
title('RGB extrinsics');

load('calibration/ir.mat');
IRParams = cameraParams;
figure(image_counter);
image_counter = image_counter + 1;
showExtrinsics(IRParams);
title('IR extrinsics');

stereoParams = stereoParameters(RGBParams,IRParams); % not enough input...
figure(image_counter);
image_counter = image_counter + 1;
showExtrinsics(stereoParams);
title('stereo extrinsics');

%% Stereo Camera Calibration 
%https://www.mathworks.com/help/vision/ref/stereoparameters.html
load('calibration/stereoCalibrationParam.mat');
figure(image_counter);
image_counter = image_counter + 1;
showExtrinsics(stereoParams);
title('stereo extrinsics');

%% load stereo images
load('handshakeStereoParams.mat');
videoFileLeft = 'handshake_left.avi';
videoFileRight = 'handshake_right.avi';

%% Create Video File Readers and the Video Player
readerLeft = VideoReader(videoFileLeft);
readerRight = VideoReader(videoFileRight);
player = vision.VideoPlayer('Position', [20,200,740 560]);

%% Read and Rectify Video Frames
frameLeft = readFrame(readerLeft);
frameRight = readFrame(readerRight);
[frameLeftRect, frameRightRect] = ...
    rectifyStereoImages(frameLeft, frameRight, stereoParams);
figure(image_counter);
image_counter = image_counter + 1;
imshow(stereoAnaglyph(frameLeftRect, frameRightRect));
title('Rectified Video Frames');

%% Compute Disparity
frameLeftGray  = rgb2gray(frameLeftRect);
figure(image_counter);
image_counter = image_counter + 1;
imagesc(frameLeftGray)
title('frameLeftGray')
frameRightGray = rgb2gray(frameRightRect);
figure(image_counter);
image_counter = image_counter + 1;
imagesc(frameRightGray)
title('frameRightGray')
    
disparityMap = disparitySGM(frameLeftGray, frameRightGray);
figure(image_counter);
image_counter = image_counter + 1;
imshow(disparityMap, [0, 64]);
title('Disparity Map');
colormap jet
colorbar

%% Reconstruct the 3-D Scene
points3D = reconstructScene(disparityMap, stereoParams);

% Convert to meters and create a pointCloud object
points3D = points3D ./ 1000;
ptCloud = pointCloud(points3D, 'Color', frameLeftRect);

% Create a streaming point cloud viewer
player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y', ...
    'VerticalAxisDir', 'down');

% Visualize the point cloud
view(player3D, ptCloud);