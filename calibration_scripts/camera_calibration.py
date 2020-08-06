from __future__ import print_function

import numpy as np
import cv2
import glob
from utils import *


class CameraCalibration(object):
    '''Calibrate single camera parameters for RGB camera and ToF camera; Calibrate stereo parameters.'''
    def __init__(self, square_size, width, height):
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.criteria_cal = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((height*width, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2) * square_size

        # Arrays to store object points and image points from all the images.
        self.objpoints = []  # 3d point in real world space
        self.imgpoints_rgb = []  # 2d points in image plane.
        self.imgpoints_tof = []  # 2d points in image plane.
        self.img_shape = (640, 480)
        self.square_size = square_size
        self.width = width
        self.height = height

        self.rgb = dict({'M': None, 'D': None})
        self.tof = dict({'M': None, 'D': None})
        self.stereo = dict({'R': None, 'T': None})


    def find_corners(self, gray, ifVisualization=False):
        '''Find corners in the img.'''
        ret, corners = cv2.findChessboardCorners(
            gray, (self.width, self.height), None)  # corners: 1 x 2 matrix
        if ret == False:
            print("Corners not found")
        else:
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            if ifVisualization:
                cornerimg = gray.copy()
                cornerimg = cv2.drawChessboardCorners(
                    cornerimg, (self.width, self.height), corners, ret)
                cv2.imshow('chessboard', cornerimg)
                cv2.waitKey(0)
        return ret, corners


    def read_images(self, filepath, width, height):
        '''Load rgb and ir images and detect corners for each image.'''
        print('Reading images...\n')
        images_rgb = glob.glob(filepath + '/rgb/*.png')
        images_tof = glob.glob(filepath + '/ir/*.png')
        images_rgb.sort()
        images_tof.sort()

        for i, fname in enumerate(images_tof):
            img_rgb = cv2.imread(images_rgb[i])
            img_tof = cv2.imread(images_tof[i])
            gray_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
            gray_tof = cv2.cvtColor(img_tof, cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret_rgb, corners_rgb = self.find_corners(gray_rgb)
            ret_tof, corners_tof = self.find_corners(gray_tof)
            if ret_rgb is True and ret_tof is True:
                # If found, add object points, image points (after refining them)
                self.objpoints.append(self.objp)
                self.imgpoints_rgb.append(corners_rgb)
                self.imgpoints_tof.append(corners_tof)


    def single_calibration(self):
        '''Calibrate for single camera.'''
        print('Start single camera calibration...\n')
        # RGB camera
        retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(self.objpoints,self.imgpoints_rgb,self.img_shape,None,None)
        hL, wL = [480, 640] # hL,wL= ChessImaL.shape[:2]
        OmtxL, roiL= cv2.getOptimalNewCameraMatrix(mtxL,distL,(wL,hL),1,(wL,hL))

        # ToF camera
        retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(self.objpoints,self.imgpoints_tof,self.img_shape,None,None)
        hR, wR = [480, 640] # hR,wR= ChessImaR.shape[:2]
        OmtxR, roiR= cv2.getOptimalNewCameraMatrix(mtxR,distR,(wR,hR),1,(wR,hR))

        self.rgb['M'] = mtxL
        self.rgb['D'] = distL
        self.tof['M'] = mtxR
        self.tof['D'] = distR


    def stereo_calibration(self):
        '''Calibrate for stereo parameters.'''
        print('Start stereo calibration...\n')
        M1 = self.rgb['M']
        d1 = self.rgb['D']
        M2 = self.tof['M']
        d2 = self.tof['D']
        flags = cv2.CALIB_FIX_INTRINSIC
        stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
        ret, M1, d1, M2, d2, R, T, E, F = cv2.stereoCalibrate(
            self.objpoints, self.imgpoints_rgb, self.imgpoints_tof, M1, d1, M2, d2, 
            self.img_shape, criteria=stereocalib_criteria, flags=flags)

        self.stereo['R'] = R
        self.stereo['T'] = T

    
    def load_cameraModel(self, rgbModel, tofModel, stereoModel):
        '''Load previously calibrated camera model.'''
        self.rgb['M'], self.rgb['D'] = load_coefficients('./rgbCamera.yml')
        self.tof['M'], self.tof['D'] = load_coefficients('./irCamera.yml')
        self.stereo['R'], self.stereo['T'] = load_coefficients('stereo.yml')


if __name__ == '__main__':
    square_size = 26.3571
    width = 6
    height = 5
    calibration = CameraCalibration(square_size, width, height)

    filepath = '../data/calibration' # contain two folders "rgb" and "ir"
    calibration.read_images(filepath, width, height)
    calibration.single_calibration()
    calibration.stereo_calibration()

    print('Saving...')
    save_rgb = 'rgbCamera.yml'
    save_coefficients(calibration.rgb['M'], calibration.rgb['D'], save_rgb)
    print('Intrinsic matrix and distortion for RGB camera is saved in ', save_rgb)
    save_tof = 'irCamera.yml'
    save_coefficients(calibration.tof['M'], calibration.tof['D'], save_tof)
    print('Intrinsic matrix and distortion for ToF camera is saved in ', save_tof)
    save_stereo = 'stereo.yml'
    save_coefficients(calibration.stereo['R'], calibration.stereo['T'], save_stereo)
    print('Rotation matrix and transformation vector for stereo camera is saved in ', save_rgb)
