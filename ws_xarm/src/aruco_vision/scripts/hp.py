#!/usr/bin/env python3

import socket
import time
import sys
import cv2
import mediapipe as mp
import numpy as np
import threading
import rospy 
from std_msgs.msg import Float32, Float32MultiArray
import math as m
import time as t
from geometry_msgs.msg import Twist




def main():
  #mediapipe settings
  rospy.init_node('hand_vel_publish', anonymous=False)
  pose_pub = rospy.Publisher("/hand_velocity", Twist, queue_size=0)
  
  mp_hands = mp.solutions.hands
  mp_drawing = mp.solutions.drawing_utils
  mp_drawing_styles = mp.solutions.drawing_styles
  hands = mp_hands.Hands(
  static_image_mode=False,
  max_num_hands=1,
  min_detection_confidence=0.7)

  cap = cv2.VideoCapture(0)

  mp_hands = mp.solutions.hands
  mp_drawing = mp.solutions.drawing_utils
  mp_drawing_styles = mp.solutions.drawing_styles
  hands = mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.7)

  pastx = 0
  pasty = 0
  pastz = 0
  dt = 0.01

  while cap.isOpened():
    time.sleep(dt)
    success, image = cap.read()
    height, width, _ = image.shape
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(image)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
          #mp_drawing.draw_landmarks(image,hand_landmarks,mp_hands.HAND_CONNECTIONS,mp_drawing_styles.get_default_hand_landmarks_style(),mp_drawing_styles.get_default_hand_connections_style())
          centerx = int(hand_landmarks.landmark[9].x * width)
          centery = int(hand_landmarks.landmark[9].y * height)
          
          distx = int(hand_landmarks.landmark[0].x * width) - centerx
          disty = int(hand_landmarks.landmark[0].y * height) - centery
          
          #cv2.line(image, (centerx, centery), (int(hand_landmarks.landmark[0].x * width), int(hand_landmarks.landmark[0].y * height)), (255, 0, 0), 2)

          posez = 6000/np.sqrt(distx**2 + disty**2) # xd
          posex = (width/2  - centerx)/14
          posey = (height/2 - centery)/14
          
          # draw center with changing color depending on distance, range of distance is 0-100
          cv2.circle(image, (centerx, centery), 10, (0, int(255*posez/100), int(255*(100-posez)/100), 0), -1)
          # draw line from center of opencv image (width/2, height/2) frame to hand, posex axis is blue, posey axis is red
          cv2.line(image, (int(width/2), int(height/2)), (centerx, centery), (int(255*(1-posex)), 0, int(255*posey)), 2)

          velx = (posex - pastx)/dt
          vely = (posey - pasty)/dt
          velz = (posez - pastz)/dt
          # print x,y,z
          print(int(posex), int(posey), int(posez))
          print(int(velx), int(vely), int(velz))
          msg = Twist()
          kpx = 1
          kpy = 1
          kpz = 0.2
          msg.linear.x = velz * kpz
          msg.linear.y =  velx * kpx
          msg.linear.z = vely * kpy
          msg.angular.x = 0.0
          msg.angular.y = 0.0
          msg.angular.z = 0.0
          pose_pub.publish(msg)
          # publish x,y,z
          
          pastx = posex
          pasty = posey
          pastz = posez

    cv2.imshow('MediaPipe Hands', image)
    cv2.waitKey(1)
    if cv2.waitKey(25) & 0xFF == ord('c'):
      break

  cap.release()
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main()
