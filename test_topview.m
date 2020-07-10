%% Initialize
clear;close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));
fileID = fopen('result_top.txt','a');

%% Choose dataset
trueV=160*318*300; % Change trueV with the dataset
numpics=2; % #of imgs to test
groundtruth=[70 80 90];
numtest=size(groundtruth,2);
results{numtest}=zeros(numpics,3);

%% Choose bias method
bias_method=2; % 0=without bias process; 1=pixel-wise; 2=linear-bias
fprintf(fileID,'bias_method=%d\n',bias_method);

%% Calculate
for num=1:size(groundtruth,2)
% for num=1:1
    for idx=1:numpics
        % filename=['/data/data0618_1/DepthImage_' num2str(idx-1,'%d'), '.png'];
        filename=['/data/fix/fix' num2str(groundtruth(num),'%d') '/DepthImage_' num2str(idx-1,'%d'), '.png'];
        D = imread(strcat(pwd, filename));
        fprintf(fileID,filename);
        fprintf(fileID,'\n');
        
        % calculate dimension
        [dimension,image_counter]=calculate_dimension(image_counter,D,bias_method);
        results{num}(idx,:)=dimension;
        
        % print result
        fprintf(fileID,'Length=%.0f, Width=%.0f, Height=%.0f\n',dimension(2),dimension(3),dimension(1));
        vol=dimension(2)*dimension(3)*dimension(1);
        error=(vol-trueV)/trueV;
        fprintf(fileID,'Error=%.1f\n\n',error*100);
    end
end

fprintf(fileID,'-----------------------\n\n\n');
fclose(fileID);