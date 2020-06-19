%% stereo calibration using toolbox
addpath(pwd);
addpath(strcat(pwd,'/data'));
addpath(strcat(pwd,'/calibration'));
folder1 = strcat(pwd, '/data/calibration0618/rgbb');
folder2 = strcat(pwd, '/data/calibration0618/tof');
squareSize = 13.1923;
stereoCameraCalibrator(folder1, folder2, squareSize);

cameraCalibrator(folder1,squareSize);

%% stereo calibration using parameters of each camera
addpath(pwd);
addpath(strcat(pwd,'/data'));
addpath(strcat(pwd,'/calibration'));
rgb=load('calibration/rgb0618.mat');
tof=load('calibration/tof0618.mat');
rotationOfCamera2 = [ 9.9996888238405357e-01, 4.9658030303482616e-03, 6.1298502306904173e-03;
    -4.8677880138721791e-03, 9.9986169860079599e-01, -1.5902462418852185e-02;
    -6.2079709598961298e-03, 1.5872128760654251e-02, 9.9985475776492971e-01 ];
translationOfCamera2 = [ -3.7419189660694208e+01, -8.3045818748568878e-01, 5.9651301067425795e+00 ];
stereoParams = stereoParameters(rgb.cameraParams, tof.cameraParams, rotationOfCamera2, translationOfCamera2);