# videoStream.py contents with CAN added
# testing image corners
import numpy as np
import cv2 as cv
import glob
import math
import time
import imutils
import can
import time 

# Candlelight firmware on Linux
bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)
msg = can.Message(arbitration_id=0x07,
                  data=[0, 25, 0, 1, 3, 1, 4, 1],
                  is_extended_id=True)

# Load previously saved data
with np.load('video.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

print(cv.__version__)
print("mtx:")
print(mtx)
print(type(mtx))
print("dist:")
print(dist)

def location(corners):
    percentage = corners[0][0][0]/1280
    width = abs(corners[0][0][0] - corners[8][0][0])
    height = abs(corners[0][0][1] - corners[3][0][1])
    return percentage, width, height

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
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280) #800, 1280
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720) #600, 720
cap.set(cv.CAP_PROP_BUFFERSIZE, 0)
#cap.set(cv.cv2.CAP_PROP_FPS, 10)
#cap = VideoStream(src=0).start()
# pixel length = 212.6, distance = 
known_distance = 67 # cm
known_length = 15.5 # cm
pixel_length = 212.6
focal_length = (pixel_length*known_distance)/known_length

while(True):
    #time.sleep(.5)
    # Capture frame-by-frame
    ret1, frame = cap.read()
    #frame = imutils.resize(frame, width=400)
    # Our operations on the frame come here
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    ret2, corners = cv.findChessboardCorners(gray, (row,col),None) #worked!
    #print("corners")
    #print(corners)
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((col*row,3), np.float32)
    objp[:,:2] = np.mgrid[0:row,0:col].T.reshape(-1,2)
    axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)

    if ret2 == True:
        corners2 = cv.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
       
        length = abs(corners2[0][0][1] - corners2[3][0][1])
    
        print("distance:")
        distance = distance_to_camera(known_length, focal_length, length)
        print(distance)

        # Find the rotation and translation vectors.
        try:
            ret,rvecs,tvecs, inliers = cv.solvePnPRansac(objp, corners2, mtx, dist) #try Ransac
        except:
            print("error")
            continue
        # Convert 3x1 rotation vector to rotation matrix for further computation            
        rotation_matrix, jacobian = cv.Rodrigues(rvecs)
        tvecs_new = -np.matrix(rotation_matrix).T * np.matrix(tvecs)

         # Projection Matrix
        pmat = np.hstack((rotation_matrix, tvecs)) # [R|t]
       
        roll, pitch, yaw = cv.decomposeProjectionMatrix(pmat)[-1]
        print(roll)
        x_distance = math.cos(roll*math.pi/180)*distance
        y_distance = abs(math.sin(roll*math.pi/180)*distance)
        print("X DISTANCE:")
        print(x_distance)
        print("Y DISTANCE:")
        print(y_distance) #bad values!
        #print("giving me bad values!")

        #print('Roll: {:.2f}\nPitch: {:.2f}\nYaw: {:.2f}'.format(float(roll), float(pitch), float(yaw)))
        # # project 3D points to image plane
        imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, mtx, dist)
        try:
            img = draw(frame,corners2,imgpts)
        except:
            print("error in draw function")
            continue

        # Send CAN message!
        msg = can.Message(arbitration_id=0x07,
                      data=[int(roll), int(x_distance), int(y_distance), 0, 0, 0, 0, 0],
                      is_extended_id=True)
        try:
          bus.send(msg)
          print("Message sent on {}".format(bus.channel_info))
        except can.CanError:
          print("Message NOT sent")

    # Show image 
    frame = imutils.resize(frame, width=800)
    cv.imshow("output", frame)  
    key = cv.waitKey(1) & 0xFF

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
    
    
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
