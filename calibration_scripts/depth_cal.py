import numpy as np
import cv2
import glob
import os
import matplotlib.pyplot as plt

from openCVcalibration import *
from test_stereo import *

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def draw(img, corner, imgpts):
    c = tuple(corner.ravel())
    img = cv2.line(img, c, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    img = cv2.line(img, c, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv2.line(img, c, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img


def visual_tf(img, objp_i, axis, rvecs, tvecs, mtx, dist, corners2_i):
    imgMark = img.copy()
    imgpts, _ = cv2.projectPoints(
        objp_i+axis, rvecs, tvecs, mtx, dist)  # objp[i]: 3 vector
    imgMark = draw(imgMark, corners2_i, imgpts)
    cv2.imshow('imgMark', imgMark)
    cv2.waitKey(0)


def read_tof(tofimg, i, j, ifDivide16):
    # corners2: x, y!!! not row, col!!!
    tof = tofimg[i, j]
    print("tof image pixel value original: ", bin(tof))
    if ifDivide16:
        tof /= 16.0
    print("tof image pixel value: ", tof)
    return tof


def expand_corners(i, width, height, objp, square_size, rvecsM, tvecs, mtx, dist, tofimg, ifDivide16):
    """ project 4 corners to image plane """
    gtDist = 0
    tof = 0
    ret = False
    if i == 0 or i == width - 1 or i == width * (height - 1) or i == width * height - 1:
        if i == 0:
            print("-------------- top left corner --------------")
            objptl = objp[i].reshape(
                (3, 1)) + np.float32([-square_size, -square_size, 0]).reshape(3, 1)
        elif i == width - 1:
            print("-------------- top right corner --------------")
            objptl = objp[i].reshape(
                (3, 1)) + np.float32([square_size, -square_size, 0]).reshape(3, 1)
        elif i == width * (height - 1):
            print("-------------- bottom left corner --------------")
            objptl = objp[i].reshape(
                (3, 1)) + np.float32([-square_size, square_size, 0]).reshape(3, 1)
        elif i == width * height - 1:
            print("-------------- bottom right corner --------------")
            objptl = objp[i].reshape(
                (3, 1)) + np.float32([+square_size, square_size, 0]).reshape(3, 1)

        pt3 = rvecsM.dot(objptl) + tvecs
        print("This corner is from ", objptl.reshape(
            (1, 3)), " to ", pt3.reshape((1, 3)))
        gtDist = np.linalg.norm(pt3)
        print("ground truth distance: ", gtDist)

        pt2d, _ = cv2.projectPoints(
            pt3, np.identity(3), np.zeros((3, 1)), mtx, dist)
        print("projected corner coordinate: ", pt2d)  # 1 x 1 x 2 matrix

        tof = read_tof(tofimg, int(
            pt2d[0][0][1]), int(pt2d[0][0][0]), ifDivide16)
        ret = True
    return ret, gtDist, tof


def cv_dist(irimg, width, height, square_size, mtx, dist, ifVisualization=False):
    cvDistances = []
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width,
                           0:height].T.reshape(-1, 2) * square_size  # n x 3 matrix
    axis_length = 2 * square_size
    axis = np.float32([[axis_length, 0, 0], [0, axis_length, 0], [
                      0, 0, -axis_length]]).reshape(-1, 3)

    gray = cv2.cvtColor(irimg, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(
        gray, (width, height), None)  # corners: 1 x 2 matrix
    if ret == False:
        print("Corners not found")
    else:
        corners_ir = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria)
        if ifVisualization:
            cornerimg = irimg.copy()
            cornerimg = cv2.drawChessboardCorners(
                cornerimg, (width, height), corners_ir, ret)
            cv2.imshow('chessboard', cornerimg)
            cv2.waitKey(0)

        """ Find the rotation and translation vectors. """
        ret, rvecs, tvecs = cv2.solvePnP(objp, corners_ir, mtx, dist)
        print("translation: ", tvecs)
        rvecsM, _ = cv2.Rodrigues(rvecs)
        print("rotation matrix: ", rvecsM)

        """ project 3D corners to image plane """
        for i in range(len(corners_ir)):
            # print("-------------- corner ", i, " --------------")
            pt3 = rvecsM.dot(objp[i].reshape((3, 1))) + tvecs
            # print("This corner is from ",
            #       objp[i], " to ", pt3.reshape((1, 3)))
            gtDist = np.linalg.norm(pt3)
            # print("ground truth distance: ", gtDist)
            cvDistances.append(gtDist)

            # print("2D Corner coordinate: ", corners_ir[i])
            # pt2d, _ = cv2.projectPoints(
            #     pt3, np.identity(3), np.zeros((3, 1)), mtx, dist)
            # print("projected corner coordinate: ", pt2d)

            if ifVisualization:
                visual_tf(irimg, objp[i], axis, rvecs,
                          tvecs, mtx, dist, corners_ir[i])
    cv2.destroyAllWindows()
    return cvDistances, corners_ir


def calibrateDepth(square_size, mtx, dist, width=6, height=5,
                   irPrefix='GrayImage_', depthPrefix='DepthImage_', rgbPrefix='RGBImage_',
                   imgID='*', image_format='png', dirpath='../data/calibration0712/plane*',
                   ifVisualization=False, ifDivide16=False, ifUseRGB=True):
    gtDistances = []
    tofs = []
    stereo_distances = []
    """ Apply camera calibration operation for images in the given directory path. """

    numfiles = len(glob.glob(dirpath + '/' + irPrefix +
                             imgID + '.' + image_format))
    print("There are ", numfiles, " files")
    for fname in glob.glob(dirpath + '/' + irPrefix + imgID + '.' + image_format):
        print("---------------------------- image ",
              fname, " ----------------------------")
        fileID = (fname.split('_')[-1]).split('.')[0]
        print(fileID)
        path, _ = os.path.split(fname)
        tofimg = cv2.imread(path + '/' + depthPrefix +
                            fileID + '.' + image_format, -1)
        print("tof image type: ", tofimg.dtype)

        irimg = cv2.imread(fname)
        cvDistances, corners_ir = cv_dist(
            irimg, width, height, square_size, mtx, dist, ifVisualization=ifVisualization)
        # print("cv distances: ", cvDistances)
        gtDistances += cvDistances

        for i in range(len(corners_ir)):
            print("-------------- corner ", i, " --------------")
            print("cv distances: ", cvDistances[i])
            tofs.append(read_tof(tofimg, int(corners_ir[i][0][1]), int(
                corners_ir[i][0][0]), ifDivide16))

        if ifUseRGB:
            rgbimg = cv2.imread(path + '/' + rgbPrefix +
                                fileID + '.' + image_format, -1)
            print("rgb image type: ", rgbimg.dtype)
            rgbgray = cv2.cvtColor(rgbimg, cv2.COLOR_BGR2GRAY)
            retRGB, cornersRGB = cv2.findChessboardCorners(
                rgbgray, (width, height), None)

            if retRGB:
                cornersRGB2 = cv2.cornerSubPix(
                    rgbgray, cornersRGB, (11, 11), (-1, -1), criteria)
                if ifVisualization:
                    cornerimg = rgbimg.copy()
                    cornerimg = cv2.drawChessboardCorners(
                        cornerimg, (width, height), cornersRGB2, retRGB)
                    cv2.imshow('chessboard RGB', cornerimg)
                    cv2.waitKey(0)
                R, T = load_coefficients('./stereo.yml')
                stereo_dists = stereo_depth(
                    rgbIntrinsic, rgbDist, mtx, dist, R, T, cornersRGB2, corners_ir)
                for i in range(len(stereo_dists)):
                    print("cv distances: ", cvDistances[i])
                    print("stereo distance: ", stereo_dists[i])
                    stereo_distances.append(stereo_dists[i])

    return gtDistances, tofs, stereo_distances


if __name__ == '__main__':
    # Load previously saved data
    irIntrinsic, irDist = load_coefficients('irCamera.yml')
    print("TOF Camera Intrinsic Matrix: ", irIntrinsic)

    rgbIntrinsic, rgbDist = load_coefficients('rgbCamera.yml')
    print("RGB Camera Intrinsic Matrix: ", rgbIntrinsic)

    square_size = 26.3571  # 13.1923  #26.3571
    width = 6  # 13 #6
    height = 5  # 6 #5

    allgtDistances = []
    alltofs = []
    allstereo_distances = []

    dirpath = '../data/calibration0720/corner*'
    gtDistances, tofs, stereo_distances = calibrateDepth(square_size=square_size, mtx=irIntrinsic, dist=irDist,
                                                         irPrefix='Gray_', depthPrefix='DepthImage_', rgbPrefix='RBG_',
                                                         width=width, height=height, imgID='*', dirpath=dirpath,
                                                         ifVisualization=False, ifDivide16=True, ifUseRGB=True)
    allgtDistances += gtDistances
    alltofs += tofs
    allstereo_distances += stereo_distances

    # dirpath = '../data/parallelPlaneTocheckerboard/Plane50'
    # gtDistances, tofs = calibrateDepth(square_size=30, mtx=irIntrinsic, dist=irDist,
    #                                    width=7, height=5, imgID='*', dirpath=dirpath,
    #                                    ifVisualization=True, ifDivide16=True, ifUseRGB=False)
    # allgtDistances += gtDistances
    # alltofs += tofs

    # dirpath = '../data/calibration0712/plane*'
    # gtDistances, tofs = calibrateDepth(square_size=square_size, mtx=irIntrinsic, dist=irDist,
    #                                    width=width, height=height, imgID='*', dirpath=dirpath, ifVisualization=False)
    # allgtDistances += gtDistances
    # alltofs += tofs

    # dirpath = '../data/calibration0716/1'
    # gtDistances, tofs = calibrateDepth(square_size=square_size, mtx=irIntrinsic, dist=irDist,
    #                                    width=width, height=height, imgID='*', dirpath=dirpath, ifVisualization=True)
    # allgtDistances += gtDistances
    # alltofs += tofs

    # dirpath = '../data/calibration0716/2'
    # gtDistances, tofs = calibrateDepth(square_size=square_size, mtx=irIntrinsic, dist=irDist,
    #                                    width=width, height=height, imgID='*', dirpath=dirpath, ifVisualization=True)
    # allgtDistances += gtDistances
    # alltofs += tofs

    plt.plot(alltofs, allgtDistances, 'ro', label='gt vs. tof')
    plt.plot(alltofs, allstereo_distances, 'yo', label='stereo vs. tof')

    A = np.vstack([alltofs, np.ones(len(alltofs))]).T
    m, c = np.linalg.lstsq(A, allgtDistances, rcond=None)[0]
    print("----------- Result: y = ", m, " * x + ", c)
    plt.plot(alltofs, m * np.array(alltofs) + c,
             'b', label='Fitted line')
    plt.legend()
    plt.xlabel('tof camera value')
    plt.ylabel('ground truth value')
    plt.show()

    save_file = 'tofCamera.yml'
    cv_file = cv2.FileStorage(save_file, cv2.FILE_STORAGE_WRITE)
    cv_file.write("m", m)
    cv_file.write("c", c)
    # gt = m * tof + c
    cv_file.release()  # note you *release* you don't close() a FileStorage object
