%% calculate every image
clear;close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));

numpics=5;
groundtruth=[70 80 90 100];
numtest=size(groundtruth,2);
results{numtest}=zeros(numpics,7);
for num=1:size(groundtruth,2)
    for idx=1:numpics
        filename=['/data/fix/fix' num2str(groundtruth(num),'%d') '/DepthImage_' num2str(idx-1,'%d'), '.png'];
        D = imread(strcat(pwd, filename));
        figure(image_counter);
        image_counter=image_counter+1;
        hold on
        results{num}(idx,:)=dimension_calculation(D);
        xlim([0 640])
        ylim([0 480])
        title(['fix' num2str(groundtruth(num))]);
        hold off;
    end
end