from __future__ import print_function

import numpy as np
import cv2
import glob
import yaml
from openCVcalibration import load_coefficients
from stereo_match import write_ply

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''

class StereoCalibration(object):
    def __init__(self, filepath,square_size, height, width):
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.criteria_cal = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((height*width, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2) * square_size

        # Arrays to store object points and image points from all the images.
        self.objpoints = []  # 3d point in real world space
        self.imgpoints_rgb = []  # 2d points in image plane.
        self.imgpoints_tof = []  # 2d points in image plane.
        self.img_shape = (640, 480)
        self.camera_model = dict({
            'leftMatrix': None, 'leftDist': None, 'leftROI': None, 'leftMap': None, 'leftStereo': None,
            'rightMatrix': None, 'rightDist': None, 'rightROI': None, 'rightMap': None, 'rightStereo': None,
            'R': None, 'T': None, 'E': None, 'F': None, 'disparityToDepthMap': None, 'wls_filter': None
        })

        self.read_images(filepath, width, height)
        M1, d1, roiL, M2, d2, roiR = self.single_calibration()
        self.camera_model['leftMatrix'] = M1
        self.camera_model['leftDist'] = d1
        self.camera_model['leftROI'] = roiL
        self.camera_model['rightMatrix'] = M2
        self.camera_model['rightDist'] = d2
        self.camera_model['rightROI'] = roiR
        # use fixed
        # self.camera_model['leftMatrix'], self.camera_model['leftDist'] = load_coefficients('./rgbCamera.yml')
        # self.camera_model['rightMatrix'], self.camera_model['rightDist'] = load_coefficients('./irCamera.yml')

        R, T, E, F = self.stereo_calibrate()
        self.camera_model['R'] = R
        self.camera_model['T'] = T
        self.camera_model['E'] = E
        self.camera_model['F'] = F
        # use fixed
        # self.camera_model['R'], self.camera_model['T'] = load_coefficients('stereo.yml')


        Q, Left_Stereo_Map, Right_Stereo_Map, stereo, stereoR, wls_filter = self.rectify()
        self.camera_model['disparityToDepthMap'] = Q
        self.camera_model['leftMap'] = Left_Stereo_Map
        self.camera_model['rightMap'] = Right_Stereo_Map
        self.camera_model['leftStereo'] = stereo
        self.camera_model['rightStereo'] = stereoR
        self.camera_model['wls_filter'] = wls_filter

        print('leftMatrix\n', self.camera_model['leftMatrix'])
        print('rightMatrix\n', self.camera_model['rightMatrix'])
        print('leftDist\n', self.camera_model['leftDist'])
        print('rightDist\n', self.camera_model['rightDist'])
        print('R\n', self.camera_model['R'])
        print('T\n', self.camera_model['T'])

        self.compute_depth()


    def read_images(self, filepath, width, height):
        print('\nStart reading images...')
        images_rgb = glob.glob(filepath + '/rgb/*.png')
        images_tof = glob.glob(filepath + '/ir/*.png')
        images_rgb.sort()
        images_tof.sort()

        for i, fname in enumerate(images_tof):
            img_rgb = cv2.imread(images_rgb[i])
            img_tof = cv2.imread(images_tof[i])
            gray_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
            gray_tof = cv2.cvtColor(img_tof, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret_rgb, corners_rgb = cv2.findChessboardCorners(gray_rgb, (width, height), None)
            ret_tof, corners_tof = cv2.findChessboardCorners(gray_tof, (width, height), None)

            if ret_rgb is True and ret_tof is True:
                # If found, add object points, image points (after refining them)
                self.objpoints.append(self.objp)

                cv2.cornerSubPix(gray_rgb, corners_rgb, (11, 11), (-1, -1), self.criteria)
                self.imgpoints_rgb.append(corners_rgb)
                # Draw and display the corners
                # img_rgb = cv2.drawChessboardCorners(img_rgb, (width, height), corners_rgb, ret_rgb)
                # cv2.imshow(images_rgb[i], img_rgb)
                # cv2.waitKey(500)

                cv2.cornerSubPix(gray_tof, corners_tof, (11, 11), (-1, -1), self.criteria)
                self.imgpoints_tof.append(corners_tof)
                # Draw and display the corners
                # img_tof = cv2.drawChessboardCorners(img_tof, (width, height), corners_tof, ret_tof)
                # cv2.imshow(images_tof[i], img_tof)
                # cv2.waitKey(500)


    def single_calibration(self):
        print('\nStart single camera calibration...')
        # Determine the new values for different parameters
        # RGB camera
        retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(self.objpoints,self.imgpoints_rgb,self.img_shape,None,None)
        hL, wL = [480, 640] # hL,wL= ChessImaL.shape[:2]
        OmtxL, roiL= cv2.getOptimalNewCameraMatrix(mtxL,distL,(wL,hL),1,(wL,hL))

        # ToF camera
        retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(self.objpoints,self.imgpoints_tof,self.img_shape,None,None)
        hR, wR = [480, 640] # hR,wR= ChessImaR.shape[:2]
        OmtxR, roiR= cv2.getOptimalNewCameraMatrix(mtxR,distR,(wR,hR),1,(wR,hR))
        return [mtxL, distL, roiL, mtxR, distR, roiR]


    def stereo_calibrate(self):
        print('\nStart stereo calibration...')
        M1 = self.camera_model['leftMatrix']
        d1 = self.camera_model['leftDist']
        M2 = self.camera_model['rightMatrix']
        d2 = self.camera_model['rightDist']
        flags = 0
        flags |= cv2.CALIB_FIX_INTRINSIC
        # flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
        # flags |= cv2.CALIB_USE_INTRINSIC_GUESS
        # flags |= cv2.CALIB_FIX_FOCAL_LENGTH
        # flags |= cv2.CALIB_FIX_ASPECT_RATIO
        # flags |= cv2.CALIB_ZERO_TANGENT_DIST
        # flags |= cv2.CALIB_RATIONAL_MODEL
        # flags |= cv2.CALIB_SAME_FOCAL_LENGTH
        # flags |= cv2.CALIB_FIX_K3
        # flags |= cv2.CALIB_FIX_K4
        # flags |= cv2.CALIB_FIX_K5

        stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
        ret, M1, d1, M2, d2, R, T, E, F = cv2.stereoCalibrate(
            self.objpoints, self.imgpoints_rgb, self.imgpoints_tof, M1, d1, M2, d2, 
            self.img_shape, criteria=stereocalib_criteria, flags=flags)
        return [R, T, E, F]

    
    def rectify(self):
        ''' Computes Left_Stereo_Map, Right_Stereo_Map, stereo, stereoR, wls_filter'''
        M1 = self.camera_model['leftMatrix']
        d1 = self.camera_model['leftDist']
        M2 = self.camera_model['rightMatrix']
        d2 = self.camera_model['rightDist']
        R = self.camera_model['R']
        T = self.camera_model['T']
        print('\nStart retifying...')
        rectify_scale= 0 # if 0 image croped, if 1 image nor croped
        RL, RR, PL, PR, Q, roiL, roiR = cv2.stereoRectify(M1, d1, M2, d2, self.img_shape, R, T, rectify_scale, (0,0))
        # initUndistortRectifyMap function
        Left_Stereo_Map = cv2.initUndistortRectifyMap(M1, d1, RL, PL, self.img_shape, cv2.CV_16SC2)   # cv2.CV_16SC2 this format is faster
        Right_Stereo_Map = cv2.initUndistortRectifyMap(M2, d2, RR, PR, self.img_shape, cv2.CV_16SC2)

        # Create StereoSGBM and prepare all parameters
        window_size = 3
        min_disp = 2
        num_disp = 130 - min_disp # must be divisible by 16
        stereo = cv2.StereoSGBM_create(minDisparity = min_disp, # for computing stereo correspondence using block matching algorithm
            numDisparities = num_disp, blockSize = window_size, uniquenessRatio = 10, speckleWindowSize = 100,
            speckleRange = 32, disp12MaxDiff = 5, P1 = 8*3*window_size**2, P2 = 32*3*window_size**2)
            # speckleRange: it will be implicitly multiplied by 16. Normally, 1 or 2 is good enough

        # Used for the filtered image
        stereoR=cv2.ximgproc.createRightMatcher(stereo) # Create another stereo for right this time
        
        # WLS FILTER Parameters
        lmbda = 80000
        sigma = 1.8
        visual_multiplier = 1.0
        wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=stereo)
        wls_filter.setLambda(lmbda)
        wls_filter.setSigmaColor(sigma)

        return [Q, Left_Stereo_Map, Right_Stereo_Map, stereo, stereoR, wls_filter]

    
    def compute_depth(self):
        leftMap = self.camera_model['leftMap']
        rightMap = self.camera_model['rightMap']
        stereoL = self.camera_model['leftStereo']
        stereoR = self.camera_model['rightStereo']
        wls_filter = self.camera_model['wls_filter']
        Q = self.camera_model['disparityToDepthMap']
        min_disp = 2
        num_disp = 130 - min_disp # must be divisible by 16

        print('\nStart compute depth for a stereo pair...')
        # img_rgb = cv2.imread('../data/data0618_1/RGBImage_0.png')
        # img_tof = cv2.imread('../data/data0618_1/GrayImage_0.png')
        img_rgb = cv2.imread('../data/calibration0720/boxA-still/RBG_5.png')
        img_tof = cv2.imread('../data/calibration0720/boxA-still/Gray_5.png')
        # Rectify the images on rotation and alignement
        Left_nice = cv2.remap(img_rgb,leftMap[0],leftMap[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
        Right_nice = cv2.remap(img_tof,rightMap[0],rightMap[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

        # # Draw Red lines
        # for line in range(0, int(img_rgb.shape[0]/20)): # Draw the Lines on the images Then numer of line is defines by the image Size/20
        #     Left_nice[line*20,:]= (0,0,255)
        #     Right_nice[line*20,:]= (0,0,255)
    
        # for line in range(0, int(img_tof.shape[0]/20)): # Draw the Lines on the images Then numer of line is defines by the image Size/20
        #     img_rgb[line*20,:]= (0,255,0)
        #     img_tof[line*20,:]= (0,255,0)    
            
        # # Show the Undistorted images
        # cv2.imshow('Both Images', np.hstack([Left_nice, Right_nice]))
        # cv2.waitKey(0)
        # cv2.imshow('Normal', np.hstack([img_rgb, img_tof]))
        # cv2.waitKey(0)

        # Convert from color(BGR) to gray
        grayR= cv2.cvtColor(Right_nice,cv2.COLOR_BGR2GRAY)
        grayL= cv2.cvtColor(Left_nice,cv2.COLOR_BGR2GRAY)

        # Compute the 2 images for the Depth_image
        disp = stereoL.compute(grayL,grayR).astype(np.float32)
        dispL = disp
        dispR = stereoR.compute(grayR,grayL).astype(np.float32)
        dispL = np.int16(dispL)
        dispR = np.int16(dispR)

        # Using the WLS filter
        filteredImg = wls_filter.filter(dispL,grayL,None,dispR)
        filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
        filteredImg = np.uint8(filteredImg)
        cv2.imshow('Disparity Map', filteredImg)
        cv2.waitKey(0)
        disp = ((disp.astype(np.float32)/ 16)-min_disp)/num_disp # Calculation allowing us to have 0 for the most distant object able to detect

        # Resize the image for faster executions
        # dispR = cv2.resize(disp,None,fx=0.7, fy=0.7, interpolation = cv2.INTER_AREA)

        # Filtering the Results with a closing filter
        kernel = np.ones((3,3),np.uint8)
        closing = cv2.morphologyEx(disp,cv2.MORPH_CLOSE, kernel) # Apply an morphological filter for closing little "black" holes in the picture(Remove noise) 

        # Colors map
        dispc = (closing-closing.min())*255
        dispC = dispc.astype(np.uint8) # Convert the type of the matrix from float32 to uint8, this way you can show the results with the function cv2.imshow()
        disp_Color = cv2.applyColorMap(dispC,cv2.COLORMAP_OCEAN) # Change the Color of the Picture into an Ocean Color_Map
        filt_Color = cv2.applyColorMap(filteredImg,cv2.COLORMAP_OCEAN) 

        # Show the result for the Depth_image
        cv2.imshow('Disparity', disp)
        cv2.waitKey(0)
        cv2.imshow('Closing',closing)
        cv2.waitKey(0)
        cv2.imshow('Color Depth',disp_Color)
        cv2.waitKey(0)
        cv2.imshow('Filtered Color Depth',filt_Color)
        cv2.waitKey(0)

        # Calculate 3D point cloud
        print('generating 3d point cloud...')
        points = cv2.reprojectImageTo3D(filteredImg,Q)  # needs to be divided by 420 to obtain metric values (80 without normalization)
        colors = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB)
        mask = disp > disp.min()
        out_points = points[mask]
        out_colors = colors[mask]
        out_fn = 'out.ply'
        write_ply(out_fn, out_points, out_colors)
        print('%s saved' % out_fn)

        print('Done')


if __name__ == '__main__':
    filepath = '../data/calibration0725/1' # contain two folders "rgb" and "tof"
    square_size = 26.3571
    height = 5
    width = 6
    cal_data = StereoCalibration(filepath,square_size,height,width)