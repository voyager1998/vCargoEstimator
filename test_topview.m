%% Initialize
clear;close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));
prefix = '/data/calibration0725/';
% select box to use
box=3; % 1=boxA, 2=boxB, 3=boxC
if box==1
    true=[158.75,316, 296.75];
    box_name='boxA';
    numpics=40;
    offset=0;
elseif box==2
    true=[275.15, 430.25, 307];
    box_name='boxB';
    offset=81;
    numpics=19;
elseif box==3
    true=[109.5, 283.5, 177.5];
    box_name='boxC';
    offset=61;
    numpics=19;
else
    error('Error: Please use correct box.\n');
end
trueV=true(1)*true(2)*true(3);
fileID = fopen(['result_' box_name '.txt'],'a'); % write results to file
% Choose bias method
bias_method=2; % 1=pixel-wise; 2=linear
fprintf(fileID,'bias_method=%d\n',bias_method);

%% Calculate
results=zeros(numpics,3);
k=1;
for idx=0:numpics
    filename = [prefix box_name '/DepthImage_' num2str(idx+offset,'%d'), '.png']; % zero index filename
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

fprintf(fileID,'-----------------------\n\n\n');
fclose(fileID);

%% visualize error
% results(:,1)=results(:,1)/0.98;
error=zeros(size(results,1),3);
for i=1:size(results,1)
    error(i,1)=((results(i,1)-true(1))/true(1))*100;
    error(i,2)=((results(i,2)-true(2))/true(2))*100;
    error(i,3)=((results(i,3)-true(3))/true(3))*100;
    error(i,4)=((results(i,1)*results(i,2)*results(i,3)-trueV)/trueV)*100;
end
x=1:1:size(error,1);
y1=ones(size(error,1));
X=[1 1 size(error,1) size(error,1)];
Y=[1 -1 -1 1];
figure;
hold on
fill(X,Y,'y')
plot(x,error(:,1),'b*--')
plot(x,error(:,2),'g*--')
plot(x,error(:,3),'r*--')
plot(x,error(:,4),'k*-')
xlabel('experiment no.')
ylabel('error %')
legend('ideal error boundary','height','length','width','volume')
title('error%');
