# open cv tutorial

import numpy as np
import cv2 as cv
import glob
row = 4
col = 3
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((col*row,3), np.float32)
objp[:,:2] = np.mgrid[0:row,0:col].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('*.jpg')
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (row,col), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (row,col), corners2, ret)
        imS = cv.resize(img, (960, 540))                    # Resize image
        cv.imshow("output", imS)                            # Show image   
        #cv.imshow('img', img)
        cv.waitKey(500)
cv.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("mtx:")
print(mtx)
print("dist:")
print(dist)
print("rvecs")
print(rvecs)
print("tvecs:")
print(tvecs)
np.savez('video.npz',mtx = mtx,dist = dist,rvecs = rvecs,tvecs = tvecs)