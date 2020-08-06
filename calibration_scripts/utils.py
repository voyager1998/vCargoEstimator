import cv2

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