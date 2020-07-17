%% Initialize
clear;close all;
image_counter = 1;
addpath(pwd);
addpath(strcat(pwd,'/utils'));

%% choose dataset
test_view=1; % 1 for top; 2 for side
fileID = fopen(['result_' test_view '.txt'],'a');

if test_view==1
    true=[160 318 300];
    numpics=10; % 14 in total
    fileID = fopen('result_top.txt','a');
    prefix='/data/top_view';
elseif test_view==2
    true=[281 433 311];
    numpics=9;
    fileID = fopen('result_side.txt','a');
    prefix='/data/side_view';
end
trueV=true(1)*true(2)*true(3);
results=zeros(numpics,3);
k=1;

%% Choose bias method
bias_method=0; % 0=without bias process; 1=pixel-wise; 2=linear-bias
fprintf(fileID,'bias_method=%d\n',bias_method);

%% Calculate
for idx=1:numpics
    filename=[prefix '/DepthImage_' num2str(idx-1,'%d'), '.png'];
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
