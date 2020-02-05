# testing image corners
import numpy as np
import cv2 as cv
import glob
import math
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
known_distance = 61 # cm
known_width = 10.2 # cm

while(True):
    # Capture frame-by-frame
    ret1, frame = cap.read()
    #frame = imutils.resize(frame, width=400)
    # Our operations on the frame come here
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    ret2, corners = cv.findChessboardCorners(gray, (row,col),None) #worked!
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((col*row,3), np.float32)
    objp[:,:2] = np.mgrid[0:row,0:col].T.reshape(-1,2)
    axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
	
    if ret2 == True:
        corners2 = cv.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        #print("corners2:")
        #print(corners2)
        #length = abs(corners2[0][0] - corners2[len(corners2)-1][0])
        #distance = distance_to_camera(known_width, focal_length, length[0])
        # Find the rotation and translation vectors.
        try:
            #ret,rvecs,tvecs, inliers = cv.solvePnPRansac(objp, corners2, mtx, dist) #try Ransac
            ret,rvecs,tvecs = cv.solvePnP(objp, corners2, mtx, dist) #try Ransac
        except:
            print("error")
            continue
        # Convert 3x1 rotation vector to rotation matrix for further computation            
        rotation_matrix, jacobian = cv.Rodrigues(rvecs)
         # Projection Matrix
        pmat = np.hstack((rotation_matrix, tvecs)) # [R|t]
        roll, pitch, yaw = cv.decomposeProjectionMatrix(pmat)[-1]
        print("Angle:")
        print(roll)
        #cos = math.cos(math.radians(roll))
        #print("real distance:")
        #print(cos * distance)

# When everything done, release the capture
cap.release()
cv.destroyAllWindows()

