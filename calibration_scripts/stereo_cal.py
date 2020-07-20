import numpy as np
import cv2
import glob
import yaml

class StereoCalibration(object):
    def __init__(self, filepath, height, width):
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS +
                         cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.criteria_cal = (cv2.TERM_CRITERIA_EPS +
                             cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((height*width, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

        # Arrays to store object points and image points from all the images.
        self.objpoints = []  # 3d point in real world space
        self.imgpoints_rgb = []  # 2d points in image plane.
        self.imgpoints_tof = []  # 2d points in image plane.

        self.cal_path = filepath
        self.height = height
        self.width = width
        self.img_shape = self.read_images(self.cal_path, self.height, self.width)

    def read_images(self, cal_path, height, width):
        images_rgb = glob.glob(cal_path + '/rgb/*.png')
        images_tof = glob.glob(cal_path + '/tof/*.png')
        images_rgb.sort()
        images_tof.sort()
        img_shape = None

        for i, fname in enumerate(images_tof):
            img_rgb = cv2.imread(images_rgb[i])
            img_tof = cv2.imread(images_tof[i])
            gray_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
            gray_tof = cv2.cvtColor(img_tof, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret_rgb, corners_rgb = cv2.findChessboardCorners(gray_rgb, (width, height), None)
            ret_tof, corners_tof = cv2.findChessboardCorners(gray_tof, (width, height), None)

            # If found, add object points, image points (after refining them)
            self.objpoints.append(self.objp)

            if ret_rgb is True:
                rt = cv2.cornerSubPix(gray_rgb, corners_rgb, (11, 11),
                                      (-1, -1), self.criteria)
                self.imgpoints_rgb.append(corners_rgb)

                # Draw and display the corners
                ret_l = cv2.drawChessboardCorners(img_rgb, (width, height),
                                                  corners_rgb, ret_rgb)
                cv2.imshow(images_rgb[i], img_rgb)
                cv2.waitKey(10)

            if ret_tof is True:
                rt = cv2.cornerSubPix(gray_tof, corners_tof, (11, 11),
                                      (-1, -1), self.criteria)
                self.imgpoints_tof.append(corners_tof)

                # Draw and display the corners
                ret_tof = cv2.drawChessboardCorners(img_tof, (width, height),
                                                  corners_tof, ret_tof)
                cv2.imshow(images_tof[i], img_tof)
                cv2.waitKey(10)
            
            img_shape = gray_rgb.shape[::-1]
            print(img_shape)

        self.camera_model = self.stereo_calibrate(img_shape)

    def stereo_calibrate(self, dims):
        flags = 0
        flags |= cv2.CALIB_FIX_INTRINSIC
        # flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
        flags |= cv2.CALIB_USE_INTRINSIC_GUESS
        flags |= cv2.CALIB_FIX_FOCAL_LENGTH
        # flags |= cv2.CALIB_FIX_ASPECT_RATIO
        flags |= cv2.CALIB_ZERO_TANGENT_DIST
        # flags |= cv2.CALIB_RATIONAL_MODEL
        # flags |= cv2.CALIB_SAME_FOCAL_LENGTH
        # flags |= cv2.CALIB_FIX_K3
        # flags |= cv2.CALIB_FIX_K4
        # flags |= cv2.CALIB_FIX_K5

        self.M1, self.d1 = load_coefficients('./rgbCamera.yml')
        self.M2, self.d2 = load_coefficients('./irCamera.yml')

        stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER +
                                cv2.TERM_CRITERIA_EPS, 100, 1e-5)
        ret, M1, d1, M2, d2, R, T, E, F = cv2.stereoCalibrate(
            self.objpoints, self.imgpoints_rgb,
            self.imgpoints_tof, self.M1, self.d1, self.M2,
            self.d2, dims,
            criteria=stereocalib_criteria, flags=flags)
        camera_model = dict([('R', R), ('T', T), ('E', E), ('F', F)])

        cv2.destroyAllWindows()

        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            self.M1, self.d1, self.M2, self.d2, dims,
            R, T, flags=cv2.CALIB_ZERO_DISPARITY)
        
        return camera_model



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
    filepath = '../data/calibration0716' # contain two folders "rgb" and "tof"
    height = 5
    width = 6
    cal_data = StereoCalibration(filepath, height, width)

# concatanate image Horizontally
# img_concate_Hori=np.concatenate((imgBGR,imgRGB),axis=1)