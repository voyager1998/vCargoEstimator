import numpy as np
import cv2
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def calibrate(square_size, width=7, height=5, prefix='GrayImage_',
              image_format='png', dirpath='../data/parallelPlaneTocheckerboard/Plane*'):
    """ Apply camera calibration operation for images in the given directory path. """
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2) * square_size
    # objp = objp * square_size

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob.glob(dirpath + '/' + prefix + '*.' + image_format)

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(
                img, (width, height), corners2, ret)
            cv2.imshow('chessboard', img)
            cv2.waitKey(100)

    cv2.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)

    return [ret, mtx, dist, rvecs, tvecs]


def save_coefficients(mtx, dist, path):
    """ Save the camera matrix and the distortion coefficients to given path/file. """
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    cv_file.write("K", mtx)  # K is intrinsic matrix
    cv_file.write("D", dist)  # D is distortion matrix
    # D = [k1, k2, p1, p2(, k3, k4, k5, k6)]
    # k is radial distortion coefficient
    # p is tangential distortion coefficient

    cv_file.release()  # note you *release* you don't close() a FileStorage object


def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]


if __name__ == '__main__':
    dirpath = '../data/calibration0716/1'
    square_size = 26.3571  # 13.1923  #26.3571
    width = 6  # 13 #6
    height = 5  # 6 #5

    # save_file = 'irCamera.yml'
    # ret, mtx, dist, rvecs, tvecs = calibrate(square_size=square_size, width=width, height=height,
    #                                          prefix='GrayImage_',
    #                                          dirpath=dirpath)
    # save_coefficients(mtx, dist, save_file)
    # print("Calibration is finished. RMS: ", ret)

    dirpath = '../data/calibration0716/*'
    save_file = 'rgbCamera.yml'
    ret, mtx, dist, rvecs, tvecs = calibrate(square_size=square_size, width=width, height=height,
                                             prefix='RGBImage_',
                                             dirpath=dirpath)
    print(tvecs)
    save_coefficients(mtx, dist, save_file)
    print("Calibration is finished. RMS: ", ret)
