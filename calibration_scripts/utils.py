import cv2
import matplotlib.pyplot as plt
import numpy as np

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


def fit_and_plot(allgtDistances, alltofs, allstereo_distances):
    '''Fit a linear relation from the ToF depth value to groundtruth depth value.'''
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
    return [m, c]


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