######################################################################################
###     camera_calibration.py
###     University of Akron NASA Robotic Mining Team
###     Source:     OpenCV calibration tutorial
###     Date:       11.13.2020
###     Authors:    Wilson Woods
###                 David Klett
######################################################################################

import numpy as np
import cv2 as cv
import glob

# number of inner corners on chessboard (row, column)
num_corners(4, 3)

# Set termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((num_corners[0] * num_corners[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:num_corners[0], 0:num_corners[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images
objpoints = []  # 3-D points in real world space
imgpoints = []  # 2-D points in image plane
images = glob.glob('*.jpg')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find chessboard corners
    corners_found, corners = cv.findChessboardCorners(gray, num_corners, None)
    # If found, add object points, image points (after refining them)
    if corners_found is True:
        # add 3-D points to object points array
        objpoints.append(objp)
        # Refine corner location to sub-pixel level accuracy
        refined_corners = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, num_corners, refined_corners, corners_found)
        imS = cv.resize(img, (960, 540))                    # Resize image
        cv.imshow("output", imS)                            # Show image
        # cv.imshow('img', img)
        cv.waitKey(500)

cv.destroyAllWindows()
# Calibrate the camera
rms, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

print("RMS Re-projection Error:")
print(rms)
print("mtx:")
print(mtx)
print("dist:")
print(dist)
print("rvecs")
print(rvecs)
print("tvecs:")
print(tvecs)

np.savez('video.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
