# testing image corners
import numpy as np
import cv2 as cv
import glob
import math
from scipy.spatial.transform import Rotation as R
import time
import imutils

# Load previously saved data
with np.load('video.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

print(cv.__version__)
print("mtx:")
print(mtx)
print("dist:")
print(dist)

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
    img = cv.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
    img = cv.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
    return img
def distance_to_camera(knownWidth, focalLength, perWidth):
	# compute and return the distance from the maker to the camera
	return (knownWidth * focalLength) / perWidth
row = 4
col = 3
cap = cv.VideoCapture(0)
#cap = cv.VideoCapture(0,cv.CAP_DSHOW)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
#cap.set(cv.cv2.CAP_PROP_FPS, 10)
#cap = VideoStream(src=0).start()
focal_length = 934.3
while(True):
    #time.sleep(.5)
    # Capture frame-by-frame
    ret1, frame = cap.read()
    #frame = imutils.resize(frame, width=400)
    # Our operations on the frame come here
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    ret2, corners = cv.findChessboardCorners(gray, (row,col),None) #worked!
    print("corners")
    print(corners)
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((col*row,3), np.float32)
    objp[:,:2] = np.mgrid[0:row,0:col].T.reshape(-1,2)
    axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

    if ret2 == True:
        corners2 = cv.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        print("corners2:")
        print(corners2)
        length = abs(corners2[0][0] - corners2[len(corners2)-1][0])
        print("length")
        print(length[0])
        known_distance = 61 # cm
        known_width = 10.2 # cm
        #focal_lenth = (length[0]*known_distance)/known_width
        print("focal_lenth:")
        print(focal_length)
        print("distance:")
        distance = distance_to_camera(known_width, focal_length, length[0])
        print(distance)

        # Find the rotation and translation vectors.
        try:
            ret,rvecs,tvecs, inliers = cv.solvePnPRansac(objp, corners2, mtx, dist) #try Ransac
        except:
            print("error")
            continue
        #r = R.from_rotvec(3,rvecs)
        # Convert 3x1 rotation vector to rotation matrix for further computation            
        rotation_matrix, jacobian = cv.Rodrigues(rvecs)
        tvecs_new = -np.matrix(rotation_matrix).T * np.matrix(tvecs)
        # print("tvecs:")
        # print(tvecs)
        # print("rvecs:")
        # print(rvecs)
        # print("tvecs_new:")
        # print(tvecs_new)
         # Projection Matrix
        pmat = np.hstack((rotation_matrix, tvecs)) # [R|t]
        # print("camera matrix:")
        # print(mtx)
        # print("[R|t]")
        # print(pmat)
        # print("dist:")
        # print(dist)
        roll, pitch, yaw = cv.decomposeProjectionMatrix(pmat)[-1]
        print(roll)
        cos = math.cos(math.radians(roll))
        print("real distance:")
        print(cos * distance)
        #print('Roll: {:.2f}\nPitch: {:.2f}\nYaw: {:.2f}'.format(float(roll), float(pitch), float(yaw)))
        # # project 3D points to image plane
        imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, mtx, dist)
        try:
            img = draw(frame,corners2,imgpts)
        except:
            print("error in draw function")
            continue
                                      
    # Show image 
    #frame = imutils.resize(frame, width=800)
    #cv.imshow("output", frame)  
    key = cv.waitKey(1) & 0xFF

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
    # # Display the resulting frame
    #frame = cv.resize(frame, (540,960))  
    #cv2.imshow('frame',frame)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
    # k = cv2.waitKey(0) & 0xFF
    # if k == ord('s'):
    #     cv2.imwrite(fname[:6]+'.png', frame)
    
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
