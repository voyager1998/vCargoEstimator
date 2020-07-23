import numpy as np
import cv2
import glob
import os
import matplotlib.pyplot as plt

from openCVcalibration import *


def stereo_depth(M_rgb, d_rgb, M_ir, d_ir, R, T, corners_rgb, corners_tof):
    """ projection matrix with distortion? """
    P1 = np.append(M_rgb, np.zeros([3, 1]), 1)
    # print('P1 is', P1)
    transformation = np.append(R, T, 1)
    P2 = M_ir.dot(transformation)
    # print('P2 is', P2)

    points4D = cv2.triangulatePoints(P1, P2, corners_rgb, corners_tof)
    points4D /= points4D[3]
    # print('4D points', points4D)

    # points3D_tof = np.dot(P2, points4D) # P2=3*4 points4D=4*30 points3D_tpf=3*30
    points3D_tof = transformation.dot(points4D)
    # print(points3D_tof)
    distances = np.linalg.norm(points3D_tof, axis=0)
    # img_depth = cv2.imread('../data/calibration0716/1/DepthImage_0.png',-1)
    # for i in range(len(corners_tof)):
    #     corner = corners_tof[i][0]
    #     x = int(corner[0])
    #     y = int(corner[1])
    #     print("corner %d (%d, %d) to (%d, %d, %d) stereo depth = %f" %
    #           (i, x, y, points3D_tof[0][i], points3D_tof[1][i], points3D_tof[2][i], distances[i]))
    return distances


def find_corners(w, h, img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(
        gray, (w, h), None)
    criteria = (cv2.TERM_CRITERIA_EPS +
                cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    if ret is True:
        cv2.cornerSubPix(gray, corners, (11, 11),
                         (-1, -1), criteria)
        # Draw and display the corners
        cv2.drawChessboardCorners(img, (w, h),
                                  corners, ret)
        cv2.imshow('rgb', img)
        cv2.waitKey(500)
    return corners


if __name__ == '__main__':
    # Load previously saved data
    irIntrinsic, irDist = load_coefficients('irCamera.yml')
    print("TOF Camera Intrinsic Matrix: ", irIntrinsic)

    rgbIntrinsic, rgbDist = load_coefficients('rgbCamera.yml')
    print("RGB Camera Intrinsic Matrix: ", rgbIntrinsic)

    RotM, Trans = load_coefficients('./stereo.yml')

    square_size = 26.3571  # 13.1923  #26.3571
    width = 6  # 13 #6
    height = 5  # 6 #5

    dirpath = '../data/calibration0720/corner380/'
    irimg = cv2.imread(dirpath + 'Gray_0.png')
    rgbimg = cv2.imread(dirpath + 'RBG_0.png')

    corners_ir = find_corners(width, height, irimg)
    corners_rgb = find_corners(width, height, rgbimg)

    distances = stereo_depth(rgbIntrinsic, rgbDist, irIntrinsic, irDist,
                             RotM, Trans, corners_rgb, corners_ir)
    print(distances)
