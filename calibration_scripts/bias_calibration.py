import numpy as np
import cv2
import glob
import os

from utils import *
from camera_calibration import CameraCalibration


def tof_dist(tofimg, cal_obj, ifDivide16):
    '''Get distance from tof camera's raw data.'''
    tofs = []
    for p in cal_obj.imgpoints_tof[-1]:
        tof = tofimg[int(p[0][1]), int(p[0][0])]
        if ifDivide16:
            tof /= 16.0
        tofs.append(tof)
    return tofs


def cv_dist(irimg, cal_obj, ifVisualization=False):
    """ Calculate distance of a corner using cv method - solvePnP."""
    cvDistances = []

    gray = cv2.cvtColor(irimg, cv2.COLOR_BGR2GRAY)
    ret, corners = cal_obj.find_corners(gray)
    if ret == True:
        cal_obj.imgpoints_tof.append(corners)
        # find the rotation and translation vectors
        ret, rvecs, tvecs = cv2.solvePnP(cal_obj.objp, corners, cal_obj.rgb['M'], cal_obj.rgb['D'])
        rvecsM, _ = cv2.Rodrigues(rvecs)

        # project 3D corners to image plane
        for i in range(len(corners)):
            pt3 = rvecsM.dot(cal_obj.objp[i].reshape((3, 1))) + tvecs
            gtDist = np.linalg.norm(pt3)
            cvDistances.append(gtDist)

            if ifVisualization:
                axis_length = 2 * calibrationObj.square_size
                axis = np.float32([[axis_length, 0, 0], [0, axis_length, 0], [
                      0, 0, -axis_length]]).reshape(-1, 3)
                visual_tf(irimg, objp[i], axis, rvecs,
                          tvecs, mtx, dist, corners_ir[i])
    cv2.destroyAllWindows()
    return ret, cvDistances


def stereo_dist(rgbimg, cal_obj):
    """ Calculate distance of a corner using stereo vision. """
    # projection matrix with distortion?
    stereoDistances = []
    rgbgray = cv2.cvtColor(rgbimg, cv2.COLOR_BGR2GRAY)
    retRGB, cornersRGB = cal_obj.find_corners(rgbgray)
    if retRGB == True:
        cal_obj.imgpoints_rgb.append(cornersRGB)

        P1 = np.append(cal_obj.rgb['M'], np.zeros([3, 1]), 1)
        transformation = np.append(cal_obj.stereo['R'], cal_obj.stereo['T'], 1)
        P2 = cal_obj.tof['M'].dot(transformation)

        points4D = cv2.triangulatePoints(P1, P2, cal_obj.imgpoints_rgb[-1], cal_obj.imgpoints_tof[-1])
        points4D /= points4D[3]

        points3D_tof = transformation.dot(points4D)
        distances = np.linalg.norm(points3D_tof, axis=0)
        for d in distances:
            stereoDistances.append(d)
    return [retRGB, stereoDistances]


def calibrateDepth(cal_obj, irPrefix='GrayImage_', depthPrefix='DepthImage_', rgbPrefix='RGBImage_',
                   imgID='*', image_format='png', dirpath='../data/calibration0712/plane*',
                   ifVisualization=False, ifDivide16=False, ifUseRGB=True):
    """ Apply camera calibration operation for images in the given directory path. """
    gtDistances = []
    tofDistances = []
    stereoDistances = []

    numfiles = len(glob.glob(dirpath + '/' + irPrefix + imgID + '.' + image_format))
    for fname in glob.glob(dirpath + '/' + irPrefix + imgID + '.' + image_format):
        fileID = (fname.split('_')[-1]).split('.')[0]
        path, _ = os.path.split(fname)
        tofimg = cv2.imread(path + '/' + depthPrefix + fileID + '.' + image_format, -1)
        irimg = cv2.imread(fname)

        # get gt distance using cv method
        ret, cv_dists = cv_dist(irimg, cal_obj, ifVisualization=ifVisualization)
        if ret == False:
            continue

        # get tof distance directly from depth image
        tof_dists = tof_dist(tofimg, cal_obj, ifDivide16)

        # get distance using stereovision
        if ifUseRGB:
            rgbimg = cv2.imread(path + '/' + rgbPrefix + fileID + '.' + image_format, -1)
            retRGB, stereo_dists = stereo_dist(rgbimg, cal_obj)
            if retRGB == False:
                continue
        gtDistances += cv_dists
        tofDistances += tof_dists
        stereoDistances += stereo_dists

    return [gtDistances, tofDistances, stereoDistances]


if __name__ == '__main__':
    square_size = 26.3571
    height = 5
    width = 6
    cal_obj = CameraCalibration(square_size, width, height)
    rgbModel = 'rgbCamera.yml'
    tofModel = 'irCamera.yml'
    stereoModel = 'stereo.yml'
    cal_obj.load_cameraModel(rgbModel, tofModel, stereoModel)

    dirpath = '../data/bias_calibration/corner*'
    gtDistances, tofs, stereo_distances = calibrateDepth(
        cal_obj, irPrefix='Gray_', depthPrefix='DepthImage_', rgbPrefix='RBG_', imgID='*', 
        dirpath=dirpath, ifVisualization=False, ifDivide16=True, ifUseRGB=True)

    m,c = fit_and_plot(gtDistances, tofs, stereo_distances)

    # save bias elimination coefficient to file
    save_file = 'tofCamera.yml'
    save_coefficients(m, c, save_file)
