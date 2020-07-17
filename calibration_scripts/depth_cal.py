import numpy as np
import cv2
import glob
import os
from openCVcalibration import load_coefficients
import matplotlib.pyplot as plt

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def draw(img, corner, imgpts):
    c = tuple(corner.ravel())
    img = cv2.line(img, c, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    img = cv2.line(img, c, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv2.line(img, c, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img


def calibrateDepth(square_size, width=6, height=5, irPrefix='GrayImage_', depthPrefix='DepthImage_',
                   imgID='*', image_format='png', dirpath='../data/calibration0712/plane*', ifVisualization=False):
    gtDistances = []
    tofs = []
    """ Apply camera calibration operation for images in the given directory path. """
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2) * square_size
    axis_length = 50
    axis = np.float32([[axis_length, 0, 0], [0, axis_length, 0], [
                      0, 0, -axis_length]]).reshape(-1, 3)

    numfiles = len(glob.glob(dirpath + '/' + irPrefix +
                             imgID + '.' + image_format))
    print("There are ", numfiles, " files")
    for fname in glob.glob(dirpath + '/' + irPrefix + imgID + '.' + image_format):
        print("---------------------------- image ",
              fname, " ----------------------------")
        # fileID = os.path.splitext(fname)[0][-1]
        fileID = fname.split('_')[-1]
        fileID = fileID.split('.')[0]
        print(fileID)
        path, _ = os.path.split(fname)
        tofimg = cv2.imread(path + '/' + depthPrefix +
                            fileID + '.' + image_format, -1)
        print(tofimg.dtype)
        if ifVisualization:
            cv2.imshow('tof image', tofimg)

        irimg = cv2.imread(fname)
        gray = cv2.cvtColor(irimg, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)
        if ret == True:
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)
            # Find the rotation and translation vectors.
            ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)
            print("translation: ", tvecs)
            rvecsM, _ = cv2.Rodrigues(rvecs)
            print("rotation matrix: ", rvecsM)
            """ project 3D corner points to image plane """
            for i in range(len(corners2)):
                print("-------------- corner ", i, " --------------")
                pt3 = rvecsM.dot(objp[i].reshape((3, 1))) + tvecs
                print("This corner is from ",
                      objp[i], " to ", pt3.reshape((1, 3)))
                gtDist = np.linalg.norm(pt3)
                print("ground truth distance: ", gtDist)
                gtDistances.append(gtDist)

                print("2D coordinate: ", corners2[i])
                # corners2: x, y!!! not row, col!!!
                tof = tofimg[int(corners2[i][0][1]), int(corners2[i][0][0])]
                print("tof image pixel value: ", tof)
                tofs.append(tof)

                if ifVisualization:
                    imgMark = irimg.copy()
                    imgpts, jac = cv2.projectPoints(
                        objp[i]+axis, rvecs, tvecs, mtx, dist)
                    imgMark = draw(imgMark, corners2[i], imgpts)
                    cv2.imshow('imgMark', imgMark)
                    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return gtDistances, tofs


if __name__ == '__main__':
    # Load previously saved data
    mtx, dist = load_coefficients('irCamera.yml')

    square_size = 26.3571  # 13.1923  #26.3571
    width = 6  # 13 #6
    height = 5  # 6 #5

    allgtDistances = []
    alltofs = []

    dirpath = '../data/calibration0712/plane*'
    gtDistances, tofs = calibrateDepth(square_size=square_size, width=width,
                                       height=height, imgID='*', dirpath=dirpath)
    allgtDistances += gtDistances
    alltofs += tofs

    dirpath = '../data/calibration0716/1'
    gtDistances, tofs = calibrateDepth(square_size=square_size, width=width,
                                       height=height, imgID='*', dirpath=dirpath)
    allgtDistances += gtDistances
    alltofs += tofs

    dirpath = '../data/calibration0716/2'
    gtDistances, tofs = calibrateDepth(square_size=square_size, width=width,
                                       height=height, imgID='*', dirpath=dirpath)
    allgtDistances += gtDistances
    alltofs += tofs

    plt.plot(alltofs, allgtDistances, 'ro')

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
