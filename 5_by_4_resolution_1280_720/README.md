# Instructions
Folder title is "5_by_4_resolution_1280_720" to indicate the checkerboard calibration.  The checkerboard is 5 by 4, and the resolution of the images is 1280 by 720. 
I found this combination is be decent.  

video_can.py calculates the camera angle (roll) and sends it over via CAN.


I followed the OpenCV tutorial to calculate the position estimation:
https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_pose/py_pose.html

I followed the camera calibration tutorial:
https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html

This tutorial shows how to generate the camera matrix (video.npz).
