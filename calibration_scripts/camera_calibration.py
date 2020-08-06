from __future__ import print_function

import numpy as np
import cv2
import glob
import yaml
from utils import *


class StereoCalibration(object):
    '''Calibrate single camera parameters for RGB camera and ToF camera; Calibrate stereo parameters.'''
    def __init__(self, filepath,square_size, height, width):
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
        self.camera_model = dict({
            'leftMatrix': None, 'leftDist': None, 'leftROI': None, 'leftMap': None, 'leftStereo': None,
            'rightMatrix': None, 'rightDist': None, 'rightROI': None, 'rightMap': None, 'rightStereo': None,
            'R': None, 'T': None, 'E': None, 'F': None, 'disparityToDepthMap': None, 'wls_filter': None
        })

        self.read_images(filepath, width, height)
        M1, d1, roiL, M2, d2, roiR = self.single_calibration()
        self.camera_model['leftMatrix'] = M1
        self.camera_model['leftDist'] = d1
        self.camera_model['leftROI'] = roiL
        self.camera_model['rightMatrix'] = M2
        self.camera_model['rightDist'] = d2
        self.camera_model['rightROI'] = roiR
        # Alternatively, uncomment the following 2 lines 
        # if already have camera parameters for each camera, can load from file
        # self.camera_model['leftMatrix'], self.camera_model['leftDist'] = load_coefficients('./rgbCamera.yml')
        # self.camera_model['rightMatrix'], self.camera_model['rightDist'] = load_coefficients('./irCamera.yml')

        R, T, E, F = self.stereo_calibrate()
        self.camera_model['R'] = R
        self.camera_model['T'] = T
        self.camera_model['E'] = E
        self.camera_model['F'] = F
        # Alternatively, uncomment the following 1 line
        # if already have stereo parameters, can load from file
        # self.camera_model['R'], self.camera_model['T'] = load_coefficients('stereo.yml')

        print('\nSaving...')
        save_rgb = 'rgbCamera.yml'
        save_coefficients(self.camera_model['leftMatrix'], self.camera_model['leftDist'], save_rgb)
        print('Intrinsic matrix and distortion for RGB camera is saved in ', save_rgb)
        save_tof = 'irCamera.yml'
        save_coefficients(self.camera_model['rightMatrix'], self.camera_model['rightDist'], save_tof)
        print('Intrinsic matrix and distortion for ToF camera is saved in ', save_tof)
        save_stereo = 'stereo.yml'
        save_coefficients(self.camera_model['R'], self.camera_model['T'], save_stereo)
        print('Rotation matrix and transformation vector for stereo camera is saved in ', save_rgb)


    def read_images(self, filepath, width, height):
        '''Load rgb and ir images and detect corners for each image.'''
        print('\nStart reading images...')
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
            ret_rgb, corners_rgb = cv2.findChessboardCorners(gray_rgb, (width, height), None)
            ret_tof, corners_tof = cv2.findChessboardCorners(gray_tof, (width, height), None)

            if ret_rgb is True and ret_tof is True:
                # If found, add object points, image points (after refining them)
                self.objpoints.append(self.objp)

                cv2.cornerSubPix(gray_rgb, corners_rgb, (11, 11), (-1, -1), self.criteria)
                self.imgpoints_rgb.append(corners_rgb)
                # Draw and display the corners
                # img_rgb = cv2.drawChessboardCorners(img_rgb, (width, height), corners_rgb, ret_rgb)
                # cv2.imshow(images_rgb[i], img_rgb)
                # cv2.waitKey(500)

                cv2.cornerSubPix(gray_tof, corners_tof, (11, 11), (-1, -1), self.criteria)
                self.imgpoints_tof.append(corners_tof)
                # Draw and display the corners
                # img_tof = cv2.drawChessboardCorners(img_tof, (width, height), corners_tof, ret_tof)
                # cv2.imshow(images_tof[i], img_tof)
                # cv2.waitKey(500)


    def single_calibration(self):
        '''Calibrate for single camera.'''
        print('\nStart single camera calibration...')
        # RGB camera
        retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(self.objpoints,self.imgpoints_rgb,self.img_shape,None,None)
        hL, wL = [480, 640] # hL,wL= ChessImaL.shape[:2]
        OmtxL, roiL= cv2.getOptimalNewCameraMatrix(mtxL,distL,(wL,hL),1,(wL,hL))

        # ToF camera
        retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(self.objpoints,self.imgpoints_tof,self.img_shape,None,None)
        hR, wR = [480, 640] # hR,wR= ChessImaR.shape[:2]
        OmtxR, roiR= cv2.getOptimalNewCameraMatrix(mtxR,distR,(wR,hR),1,(wR,hR))
        return [mtxL, distL, roiL, mtxR, distR, roiR]


    def stereo_calibrate(self):
        '''Calibrate for stereo parameters.'''
        print('\nStart stereo calibration...')
        M1 = self.camera_model['leftMatrix']
        d1 = self.camera_model['leftDist']
        M2 = self.camera_model['rightMatrix']
        d2 = self.camera_model['rightDist']
        flags = cv2.CALIB_FIX_INTRINSIC
        stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
        ret, M1, d1, M2, d2, R, T, E, F = cv2.stereoCalibrate(
            self.objpoints, self.imgpoints_rgb, self.imgpoints_tof, M1, d1, M2, d2, 
            self.img_shape, criteria=stereocalib_criteria, flags=flags)
        return [R, T, E, F]


if __name__ == '__main__':
    filepath = '../data/calibration' # contain two folders "rgb" and "tof"
    square_size = 26.3571
    height = 5
    width = 6
    cal_data = StereoCalibration(filepath,square_size,height,width)