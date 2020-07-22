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


def calibrateDepth(square_size, mtx, dist, width=6, height=5,
                   irPrefix='GrayImage_', depthPrefix='DepthImage_', rgbPrefix='RGBImage_',
                   imgID='*', image_format='png', dirpath='../data/calibration0712/plane*',
                   ifVisualization=False, ifDivide16=False, ifUseRGB=True):
    gtDistances = []
    tofs = []
    """ Apply camera calibration operation for images in the given directory path. """
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width,
                           0:height].T.reshape(-1, 2) * square_size  # n x 3 matrix
    axis_length = 2 * square_size
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
        print("tof image type: ", tofimg.dtype)

        if ifUseRGB:
            rgbimg = cv2.imread(path + '/' + rgbPrefix +
                                fileID + '.' + image_format, -1)
            print("rgb image type: ", rgbimg.dtype)
            rgbgray = cv2.cvtColor(rgbimg, cv2.COLOR_BGR2GRAY)
            retRGB, cornersRGB = cv2.findChessboardCorners(
                rgbgray, (width, height), None)

        irimg = cv2.imread(fname)
        gray = cv2.cvtColor(irimg, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(
            gray, (width, height), None)  # corners: 1 x 2 matrix
        if ret == False:
            print("Corners not found")
        else:
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)
            if ifVisualization:
                cornerimg = irimg.copy()
                cornerimg = cv2.drawChessboardCorners(
                    cornerimg, (width, height), corners2, ret)
                cv2.imshow('chessboard', cornerimg)
                cv2.waitKey(0)

            """ Find the rotation and translation vectors. """
            ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)
            # _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)
            print("translation: ", tvecs)
            rvecsM, _ = cv2.Rodrigues(rvecs)
            print("rotation matrix: ", rvecsM)

            if ifUseRGB:
                cornersRGB2 = cv2.cornerSubPix(
                    rgbgray, cornersRGB, (11, 11), (-1, -1), criteria)
                # Find the rotation and translation vectors.
                retRGB, rvecsRGB, tvecsRGB = cv2.solvePnP(
                    objp, cornersRGB2, rgbIntrinsic, rgbDist)
                print("translation in RGB: ", tvecsRGB)



            """ project 3D corners to image plane """
            for i in range(len(corners2)):
                """ project 4 corners to image plane """
                if i == 0 or i == width - 1 or i == width * (height - 1) or i == width * height - 1:
                    if i == 0:
                        print("-------------- top left corner --------------")
                        objptl = objp[i].reshape((3, 1)) + np.float32([-square_size, -square_size, 0]).reshape(3, 1)
                    elif i == width - 1:
                        print("-------------- top right corner --------------")
                        objptl = objp[i].reshape((3, 1)) + np.float32([square_size, -square_size, 0]).reshape(3, 1)
                    elif i == width * (height - 1):
                        print("-------------- bottom left corner --------------")
                        objptl = objp[i].reshape((3, 1)) + np.float32([-square_size, square_size, 0]).reshape(3, 1)
                    elif i == width * height - 1:
                        print("-------------- bottom right corner --------------")
                        objptl = objp[i].reshape((3, 1)) + np.float32([+square_size, square_size, 0]).reshape(3, 1)

                    pt3 = rvecsM.dot(objptl) + tvecs
                    print("This corner is from ", objptl.reshape((1, 3)), " to ", pt3.reshape((1, 3)))
                    gtDist = np.linalg.norm(pt3)
                    print("ground truth distance: ", gtDist)
                    gtDistances.append(gtDist)

                    pt2d, jac = cv2.projectPoints(
                        pt3, np.identity(3), np.zeros((3, 1)), mtx, dist)
                    print("projected corner coordinate: ", pt2d)  # 1 x 1 x 2 matrix

                    # corners2: x, y!!! not row, col!!!
                    tof = tofimg[int(pt2d[0][0][1]), int(pt2d[0][0][0])]
                    print("tof image pixel value original: ", bin(tof))

                    if ifDivide16:
                        tof /= 16.0
                    print("tof image pixel value: ", tof)
                    tofs.append(tof)


                print("-------------- corner ", i, " --------------")
                pt3 = rvecsM.dot(objp[i].reshape((3, 1))) + tvecs
                print("This corner is from ",
                      objp[i], " to ", pt3.reshape((1, 3)))
                gtDist = np.linalg.norm(pt3)
                print("ground truth distance: ", gtDist)
                gtDistances.append(gtDist)

                print("2D Corner coordinate: ", corners2[i])
                pt2d, jac = cv2.projectPoints(
                    pt3, np.identity(3), np.zeros((3, 1)), mtx, dist)
                print("projected corner coordinate: ", pt2d)

                # corners2: x, y!!! not row, col!!!
                tof = tofimg[int(corners2[i][0][1]), int(corners2[i][0][0])]
                print("tof image pixel value original: ", bin(tof))

                # # switch 2 channels: tof = tofc1 | tofc2
                # tofc1 = tof >> 8
                # tofc2 = (tof << 8) % (1 << 16)
                # # tof = (((tof << 8) + (tof >> 8)) % (1 << 16)) >> 4
                if ifDivide16:
                    tof /= 16.0
                print("tof image pixel value: ", tof)
                tofs.append(tof)

                if ifVisualization:
                    imgMark = irimg.copy()
                    imgpts, jac = cv2.projectPoints(
                        objp[i]+axis, rvecs, tvecs, mtx, dist) # objp[i]: 3 vector
                    imgMark = draw(imgMark, corners2[i], imgpts)
                    cv2.imshow('imgMark', imgMark)
                    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return gtDistances, tofs


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

    dirpath = '../data/calibration0720/corner*'
    gtDistances, tofs = calibrateDepth(square_size=square_size, mtx=irIntrinsic, dist=irDist,
                                       irPrefix='Gray_', depthPrefix='DepthImage_',
                                       width=width, height=height, imgID='*', dirpath=dirpath,
                                       ifVisualization=False, ifDivide16=True, ifUseRGB=False)
    allgtDistances += gtDistances
    alltofs += tofs

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
