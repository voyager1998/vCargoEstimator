%% Initialize
clear;close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));
fileID = fopen('result_side.txt','a');

%% Choose dataset
% true=[160 318 300];
true=[281 433 311];
trueV=true(1)*true(2)*true(3);
numpics=8; % #of imgs to test
groundtruth=[70 80];
numtest=size(groundtruth,2);
% results=zeros(size(groundtruth,2)*numpics,3);
results=zeros(numpics,3);
k=1;

%% Choose bias method
bias_method=2; % 0=without bias process; 1=pixel-wise; 2=linear-bias
fprintf(fileID,'bias_method=%d\n',bias_method);

%% Calculate
% for num=1:size(groundtruth,2)
for num=1:1
    for idx=1:numpics
        filename=['/data/data0618_1/DepthImage_' num2str(idx-1,'%d'), '.png'];
%         filename=['/data/fix/fix' num2str(groundtruth(num),'%d') '/DepthImage_' num2str(idx-1,'%d'), '.png'];
        D = imread(strcat(pwd, filename));
        fprintf(fileID,filename);
        fprintf(fileID,'\n');
        
        % calculate dimension
        [dimension,image_counter]=calculate_dimension(image_counter,D,bias_method);
        results(k,:)=dimension;
        k=k+1;
        
        % print result
        fprintf(fileID,'Length=%.0f, Width=%.0f, Height=%.0f\n',dimension(2),dimension(3),dimension(1));
        vol=dimension(2)*dimension(3)*dimension(1);
        error=(vol-trueV)/trueV;
        fprintf(fileID,'Error=%.1f\n\n',error*100);
    end
end

fprintf(fileID,'-----------------------\n\n\n');
fclose(fileID);

%% visualize error
error=zeros(size(results,1),3);
for i=1:size(results,1)
    error(i,1)=((results(i,1)-true(1))/true(1))*100;
    error(i,2)=((results(i,2)-true(2))/true(2))*100;
    error(i,3)=((results(i,3)-true(3))/true(3))*100;
    error(i,4)=((results(i,1)*results(i,2)*results(i,3)-trueV)/trueV)*100;
end
x=1:1:size(error,1);
figure;
hold on
plot(x,error(:,1),'b-*')
plot(x,error(:,2),'g-*')
plot(x,error(:,3),'r-*')
plot(x,error(:,4),'y-*')
xlabel('experiment no.')
ylabel('error %')
legend('height','length','width')
title('error%');
