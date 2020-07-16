import numpy as np
import cv2
import glob
from openCVcalibration import load_coefficients

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def draw(img, corner, imgpts):
    c = tuple(corner.ravel())
    img = cv2.line(img, c, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    img = cv2.line(img, c, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv2.line(img, c, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img


def calibrateDepth(square_size, width=6, height=5, irPrefix='GrayImage_',
                   image_format='png', dirpath='../data/calibration0712/plane50'):
    """ Apply camera calibration operation for images in the given directory path. """
    objp = np.zeros((height*width, 3), np.float32)
    objp[:,:2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2) * square_size
    axis_length = 50
    axis = np.float32([[axis_length, 0, 0], [0, axis_length, 0], [0, 0, -axis_length]]).reshape(-1, 3)

    for fname in glob.glob(dirpath + '/' + irPrefix + '*.' + image_format):
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)
        if ret == True:
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)
            # Find the rotation and translation vectors.
            ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)
            print(tvecs)
            """ project 3D corner points to image plane """
            for i in range(len(corners2)):
                imgMark = img.copy()
                imgpts, jac = cv2.projectPoints(objp[i]+axis, rvecs, tvecs, mtx, dist)
                imgMark = draw(imgMark, corners2[i], imgpts)
                cv2.imshow('imgMark', imgMark)
                cv2.waitKey(100)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    # Load previously saved data
    mtx, dist = load_coefficients('irCamera.yml')

    dirpath = '../data/calibration0712/plane50'
    square_size = 26.3571  # 13.1923  #26.3571
    width = 6  # 13 #6
    height = 5  # 6 #5

    calibrateDepth(square_size=square_size, width=width,
                   height=height, dirpath=dirpath)
