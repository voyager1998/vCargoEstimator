addpath(pwd);
addpath(strcat(pwd,'/data'));
folder1 = strcat(pwd, '/data/calibration0618/rgb');
folder2 = strcat(pwd, '/data/calibration0618/tof');
stereoCameraCalibrator(folder1, folder2, 13.1923);
