import numpy as np
import cv2

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
    cal_depth()