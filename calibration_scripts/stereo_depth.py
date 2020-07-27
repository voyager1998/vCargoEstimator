import numpy as np
import cv2
from openCVcalibration import load_coefficients

def cal_depth():
    M1, d1 = load_coefficients('./rgbCamera.yml')
    M2, d2 = load_coefficients('./irCamera.yml')
    R, T = load_coefficients('./stereo.yml')
    # R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify( M1, d1, M2, d2,
    #     (640, 480), R, T, flags=cv2.CALIB_ZERO_DISPARITY)
    P1 = np.append(M1, np.zeros([len(M1), 1]), 1)
    print('P1 is', P1)
    transformation = np.append(R, T, 1)
    P2 = M2.dot(transformation)
    print('P2 is', P2)
    
    corners_rgb, corners_tof = find_corners()
    # print(corners_rgb)
    points4D = cv2.triangulatePoints(P1, P2, corners_rgb, corners_tof)
    points4D /= points4D[3]
    print('4D points', points4D)

    # points3D_tof = np.dot(P2, points4D) # P2=3*4 points4D=4*30 points3D_tpf=3*30
    points3D_tof = transformation.dot(points4D)
    print(points3D_tof)
    distances = np.linalg.norm(points3D_tof, axis=0)
    img_depth = cv2.imread('../data/calibration0716/1/DepthImage_0.png',-1)
    # print(corners_tof)
    for i in range(len(corners_tof)):
        corner = corners_tof[i][0]
        x = int(corner[0])
        y = int(corner[1])
        print("corner %d (%d, %d) to (%d, %d, %d) tof depth = %f stereo depth = %f" % 
            (i, x, y, points3D_tof[0][i], points3D_tof[1][i], points3D_tof[2][i], img_depth[y][x], distances[i]))


# find corners in chessboard
def find_corners():
    width = 6
    height = 5
    img_rgb = cv2.imread('../data/calibration0716/1/RGBImage_0.png')
    img_tof = cv2.imread('../data/calibration0716/1/GrayImage_0.png')
    gray_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
    gray_tof = cv2.cvtColor(img_tof, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret_rgb, corners_rgb = cv2.findChessboardCorners(gray_rgb, (width, height), None)
    ret_tof, corners_tof = cv2.findChessboardCorners(gray_tof, (width, height), None)
    criteria = (cv2.TERM_CRITERIA_EPS +
            cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    if ret_rgb is True:
        rt = cv2.cornerSubPix(gray_rgb, corners_rgb, (11, 11),
                                (-1, -1), criteria)
        # Draw and display the corners
        ret_rgb = cv2.drawChessboardCorners(img_rgb, (width, height),
                                            corners_rgb, ret_rgb)
        cv2.imshow('rgb', img_rgb)
        cv2.waitKey(500)

    if ret_tof is True:
        rt = cv2.cornerSubPix(gray_tof, corners_tof, (11, 11),
                                (-1, -1), criteria)
        # Draw and display the corners
        ret_tof = cv2.drawChessboardCorners(img_tof, (width, height),
                                            corners_tof, ret_tof)
        cv2.imshow('tof', img_tof)
        cv2.waitKey(500)
            
    return [corners_rgb, corners_tof]


# find four corners of a box
def find_box_corners():
    img_rgb = cv2.imread('../data/calibration0725/boxA/RBG_0.png')
    gray_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
    plane_rgb = np.zeros((480, 640), np.float32)
    roi_rgb = gray_rgb[40:70,25:50]
    cv2.imshow('rgb_roi', roi_rgb)
    features1 = cv2.cornerHarris(roi_rgb,2,3,0.04)
    plane_rgb[40:70,25:50] = features1
    img_rgb[plane_rgb>0.9*plane_rgb.max()]=[0,0,255]
    cv2.imshow('rgb',img_rgb)

    img_tof = cv2.imread('../data/calibration0725/boxA/Gray_0.png')
    gray_tof = cv2.cvtColor(img_tof, cv2.COLOR_BGR2GRAY)
    plane_tof = np.zeros((480, 640), np.float32)
    roi_tof = gray_tof[140:180,150:170]
    cv2.imshow('tof_roi', roi_tof)
    features2 = cv2.cornerHarris(roi_tof,2,3,0.04)
    plane_tof[140:180,150:170] = features2
    img_tof[plane_tof>0.5*plane_tof.max()]=[0,0,255]
    cv2.imshow('tof', img_tof)

    cv2.waitKey(0)
    # return [corners_rgb, corners, tof]


if __name__ == '__main__':
    find_box_corners()