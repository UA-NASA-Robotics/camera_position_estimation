# saving video images
import time
import cv2
import keyboard
print(cv2.__version__)
#vidcap = cv2.VideoCapture(0)
vidcap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
vidcap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
vidcap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
success,image = vidcap.read()
count = 0
success = True

while success:
    #cv2.imwrite("y%d.jpg" % count, image)     # save frame as JPEG file
    success,image = vidcap.read()
    #image = imutils.resize(image, width=600)
    cv2.imshow("output", image)  
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
          break

    if keyboard.is_pressed('c'):  # if key 'c' is pressed 
        cv2.imwrite("me%d.jpg" % count, image)     # save frame as JPEG file
        print ('Save a new frame: ', success)
    count += 1
    #time.sleep(5)