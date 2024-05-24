#!/usr/bin/env python3
import cv2 # Import the OpenCV library
import numpy as np # Import Numpy library
from scipy.spatial.transform import Rotation as R
import math # Math library
import os
import rospy
from geometry_msgs.msg import Point, Twist
import time 


# Dictionary that was used to generate the ArUco marker
aruco_dictionary_name = "DICT_4X4_50"
 
# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}
 
# Side length of the ArUco marker in cm 
aruco_marker_side_length = 6
 
# Calibration parameters yaml file 
def euler_from_quaternion(x, y, z, w):
  """
  Convert a quaternion into euler angles (roll, pitch, yaw)
  roll is rotation around x in radians (counterclockwise)
  pitch is rotation around y in radians (counterclockwise)
  yaw is rotation around z in radians (counterclockwise)
  """
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll_x = math.atan2(t0, t1)
      
  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch_y = math.asin(t2)
      
  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw_z = math.atan2(t3, t4)
      
  return roll_x, pitch_y, yaw_z # in radians
 




def main():

  pose_pub = rospy.Publisher("/aruco_pose", Twist, queue_size=0)
  rospy.init_node('aruco_pose_publisher', anonymous=False)


  """
  Main method of the program.
  """
  # Check that we have a valid ArUco marker
  if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
    print("ArUCo tag is not supported")
 
  # Load the camera parameters from the saved file
  root = os.getcwd()
  calibrationDir = os.path.join(root,'calibration.npz')
  calib_data = np.load(calibrationDir)
  camMatrix = calib_data['camMatrix']
  distCoeff = calib_data['distCoeff']
  tvecs = calib_data['tvecs']
  rvecs = calib_data['rvecs']
     
  # Load the ArUco dictionary
  print("[INFO] detecting '{}' markers...".format(
    aruco_dictionary_name))
  this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_dictionary_name])
  this_aruco_parameters = cv2.aruco.DetectorParameters()
  detector = cv2.aruco.ArucoDetector(this_aruco_dictionary,this_aruco_parameters)
   
  # Start the video stream
  cap = cv2.VideoCapture(0)
  
  transform_translation_x_last = 0
  transform_translation_y_last = 0
  transform_translation_z_last = 0
  transform_translation_x_current = 0
  transform_translation_y_current = 0
  transform_translation_z_current = 0

  while(True):
  
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = cap.read()  

    if not ret:
      break

    # Detect ArUco markers in the video frame
    (corners, marker_ids, rejected) = detector.detectMarkers(frame)
       
    # Check that at least one ArUco marker was detected
    if marker_ids is not None:
 
      # Draw a square around detected markers in the video frame
      cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
       
      # Get the rotation and translation vectors
      rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        aruco_marker_side_length,
        camMatrix,
        distCoeff)
         
      # Print the pose for the ArUco marker
      # The pose of the marker is with respect to the camera lens frame.
      # Imagine you are looking through the camera viewfinder, 
      # the camera lens frame's:
      # x-axis points to the right
      # y-axis points straight down towards your toes
      # z-axis points straight ahead away from your eye, out of the camera
      for i, marker_id in enumerate(marker_ids):
        
        # # Store the translation (i.e. position) information
        transform_translation_x = tvecs[0][0][0]
        transform_translation_y = tvecs[0][0][1]
        transform_translation_z = tvecs[0][0][2]
        transform_translation_x_current = transform_translation_x
        transform_translation_y_current = transform_translation_y
        transform_translation_z_current = transform_translation_z
        vx = (transform_translation_x_current - transform_translation_x_last)/0.01
        vy = (transform_translation_y_current - transform_translation_y_last)/0.01
        vz = (transform_translation_z_current - transform_translation_z_last)/0.01
        msg = Twist()
        msg.linear.x = vy
        msg.linear.y = -vx
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        pose_pub.publish(msg)

        transform_translation_x_last = transform_translation_x_current
        transform_translation_y_last = transform_translation_y_current
        transform_translation_z_last = transform_translation_z_current
        # # Store the rotation information
        # rotation_matrix = np.eye(4)
        # rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
        # r = R.from_matrix(rotation_matrix[0:3, 0:3])
        # quat = r.as_quat()   
         
        # # Quaternion format     
        # transform_rotation_x = quat[0] 
        # transform_rotation_y = quat[1] 
        # transform_rotation_z = quat[2] 
        # transform_rotation_w = quat[3] 
         
        # # Euler angle format in radians
        # roll_x, pitch_y, yaw_z = euler_from_quaternion(transform_rotation_x, 
        #                                                transform_rotation_y, 
        #                                                transform_rotation_z, 
        #                                                transform_rotation_w)
         
        # roll_x = math.degrees(roll_x)
        # pitch_y = math.degrees(pitch_y)
        # yaw_z = math.degrees(yaw_z)
        print("transform_translation_x: {}".format(transform_translation_x))
        print("transform_translation_y: {}".format(transform_translation_y))
        print("transform_translation_z: {}".format(transform_translation_z))
        # print("roll_x: {}".format(roll_x))
        # # print("pitch_y: {}".format(pitch_y))
        # print("yaw_z: {}".format(yaw_z))
        print()
        
        # Draw the axes on the marker
        cv2.drawFrameAxes(frame, camMatrix, distCoeff, rvecs[0], tvecs[0], aruco_marker_side_length/2, 2)
     
    # Display the resulting frame
    cv2.imshow('frame',frame)
    # If "q" is pressed on the keyboard, 
    # exit this loop



    if cv2.waitKey(10) & 0xFF==ord('1'):
      break
  
  # Close down the video stream
  cap.release()
  cv2.destroyAllWindows()



if __name__ == '__main__':
  main()