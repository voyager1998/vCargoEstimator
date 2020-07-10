clear;close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));

% load file
groundtruth=90;
idx=1;
filename=['/data/fix/fix' num2str(groundtruth,'%d') '/DepthImage_' num2str(idx,'%d'), '.png'];
D = imread(strcat(pwd, filename));
% fit edge
figure(image_counter);
image_counter=image_counter+1;
hold on
result=dimension_calculation(D);
xlim([0 640])
ylim([0 480])
title(['fix' num2str(groundtruth)]);
hold off;

% calculate l/w/h and volume
v=result(1)*result(2)*result(3);
trueV=160*318*300;
error=(v-trueV)/trueV;
v=v/(100)^3;
trueV=trueV/(100)^3;
disp('Length=',num2str(result(2)));
disp('Width=',num2str(result(3)));
disp('Height=',num2str(result(1)));
disp('Vol=',num2str(v),'cm^3');
disp('Error%=',num2str(error));