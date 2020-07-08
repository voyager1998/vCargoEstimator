%% calculate every image
clear;close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));
load('calibration/panasonicIRcameraParams.mat');
C_ir = irCameraParams.IntrinsicMatrix';

numpics=1;
groundtruth=[70 80 90 100 110];
results{5}=zeros(numpics,7);
bias=load('bias.mat').p;
for num=1:size(groundtruth,2)
    for idx=1:numpics
        filename=['/data/fix/fix' num2str(groundtruth(num),'%d') '/DepthImage_' num2str(idx-1,'%d'), '.png'];
        D = imread(strcat(pwd, filename));
        D = D/16;
        D_undistort = undistortImage(D,irCameraParams);
        figure(image_counter);
        image_counter=image_counter+1;
        hold on
        results{num}(idx,:)=dimension_calculation(D_undistort,C_ir,bias);
        xlim([0 640])
        ylim([0 480])
        title(['fix' num2str(groundtruth(num))]);
        hold off;
    end
end