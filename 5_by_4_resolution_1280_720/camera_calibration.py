# open cv tutorial

import numpy as np
import cv2 as cv
import glob


row = 4
col = 3


# termination criteria
# criteria = (criteria, max_iter, epsilon)
# TERM_CRITERIA_EPS: stop the algorithm iteration if specified accuracy, epsilon, is reached
# TERM_CRITERIA_MAX_ITER - stop the algorithm after the specified number of iterations, max_iter
# TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER - stop the iteration when any of the above condition is met
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((col*row, 3), np.float32)
objp[:, :2] = np.mgrid[0:row, 0:col].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3-D points in real world space
imgpoints = []  # 2-D points in image plane
images = glob.glob('*.jpg')
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)


    # STEP 1 find chessboard corners
    # findChessboardCorners(image, patternSize, corners)
    # image = source chessboard image. must be 8-bit, grayscale or color
    # patternSize = # of inner corners for chessboard rows and columns
    # output array of detected corners
    # function will return 0 on failure to find and order corners
    corners_found, corners = cv.findChessboardCorners(gray, (row, col), None)

    # If found, add object points, image points (after refining them)
    if corners_found is True:
        # add 3-D points to object points array
        objpoints.append(objp)


        # STEP 2 refine corner location to sub-pixel level accuracy
        # cornerSubPix(image, corners, winSize, zeroZone, criteria)
        # image = single-channel, 8-bit or float image
        # corners = initial coordinates of the input corners and refined coordinates
        # provided for output
        # winSize = half the side length of the search window
            # ex: if winSize = Size(5, 5), then we have 11 x 11 search windown
        # zeroZone = half of the size of the dead region in the middle of the search zone
            # (-1, -1) indicates no such size / zone
        # criteria = criteria for termination of the iterative corner refinement
        # return: refined corner locations
        refined_corners = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)


        # STEP 3 draw and display the corners
        # drawChessboardCorners(image, patternSize, corners, patternWasFound)
        # image = destination image, must be 8-bit, color
        # patternSize = # of inner corners per chessboard row and column
        # corners = array of detected corners
            # numerical output from findChessboardCorenrs() should be passed here
        # patternWasFound = boolean indicating whether complete board was detected
            # boolean output from findChessboardCorners() should be passed here
        cv.drawChessboardCorners(img, (row, col), refined_corners, corners_found)
        imS = cv.resize(img, (960, 540))                    # Resize image
        cv.imshow("output", imS)                            # Show image
        # cv.imshow('img', img)
        cv.waitKey(500)
cv.destroyAllWindows()


# calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs)
# objectPoints = points in 3-D space
# imagePoints = points in 2-D space
# imageSize = size of image (this is only used to initialize the camera intrinsic matrix)
# camera matrix = 3 x 3 matrix containing focal length and optical centers of the camera
# see openCVNotes.txt for more
# distCoeffs = distortion coefficients (for fitting the radial/tangential distortion model)
# return: RMS re-projection error, camera matrix, distortion coeffs, rotation vectors, translation vectors
    # not that RMS should be between 0.1 and 1.0 pixels for good calibration
rms, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

print("mtx:")
print(mtx)
print("dist:")
print(dist)
print("rvecs")
print(rvecs)
print("tvecs:")
print(tvecs)
np.savez('video.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
