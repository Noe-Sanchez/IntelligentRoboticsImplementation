import cv2
import os
import glob
import pathlib
import numpy as np
import json

import time 

class threshold():
    def __init__(self):
        self.BASE_DIR = str(pathlib.Path(__file__).resolve().parent)
        self.color_dir = self.BASE_DIR + '/colors'
        self.frame = None
        self.flag = False
        self.cnt = 0

        self.current_label = ord('0')
        self.labels = self.getLabelsDict()

        self.showDict()

    def showDict(self):
        # Show labels as with its key number
        for key, value in self.labels.items():
            print(f'{chr(key)}: {value}')
        
    
    def getLabelsDict(self):
        labels = {}
        cnt = 0
        for i, file in enumerate(glob.glob(self.color_dir + '/*')):
            if file.find('png') != -1:
                continue
            labels[ord(str(cnt + 1))] = file.removeprefix(self.color_dir + '/')
            cnt += 1
        return labels
    
    def getHSV(self):
        # Get the HSV values for the colors from a json file
        try:
            with open(self.BASE_DIR + '/hsv_colors.json', 'r') as f:
                self.hsv_dict = json.load(f) 
        except FileNotFoundError:
            self.hsv_dict = {
                "yellow": [[0, 0, 0], [255, 255, 255]],
                "red": [[0, 0, 0], [255, 255, 255]],
                "green": [[0, 0, 0], [255, 255, 255]],
            }

        # Deserialize the dictionary and convert the values to numpy array
        self.hsv_dict = {key: [np.array(value[0]), np.array(value[1])] for key, value in self.hsv_dict.items()}

        # Mannualy change the values using trackbars once for each color
        cv2.namedWindow('trackbars')
        cv2.createTrackbar('Hue Min', 'trackbars', 0, 255, lambda x: None)
        cv2.createTrackbar('Hue Max', 'trackbars', 0, 255, lambda x: None)
        cv2.createTrackbar('Sat Min', 'trackbars', 0, 255, lambda x: None)
        cv2.createTrackbar('Sat Max', 'trackbars', 0, 255, lambda x: None)
        cv2.createTrackbar('Val Min', 'trackbars', 0, 255, lambda x: None)
        cv2.createTrackbar('Val Max', 'trackbars', 0, 255, lambda x: None)

        for key, value in self.labels.items():
            print (key - ord('0'), value)
            cv2.setTrackbarPos('Hue Min', 'trackbars', self.hsv_dict[value][0][0])
            cv2.setTrackbarPos('Hue Max', 'trackbars', self.hsv_dict[value][1][0])
            cv2.setTrackbarPos('Sat Min', 'trackbars', self.hsv_dict[value][0][1])
            cv2.setTrackbarPos('Sat Max', 'trackbars', self.hsv_dict[value][1][1])
            cv2.setTrackbarPos('Val Min', 'trackbars', self.hsv_dict[value][0][2])
            cv2.setTrackbarPos('Val Max', 'trackbars', self.hsv_dict[value][1][2])
            for file in glob.glob(self.color_dir + f'/{value}/*'):
                frame = cv2.imread(file)
                
                frame = cv2.resize(frame, (800,400))
                while True:
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    h_min = cv2.getTrackbarPos('Hue Min', 'trackbars')
                    h_max = cv2.getTrackbarPos('Hue Max', 'trackbars')
                    s_min = cv2.getTrackbarPos('Sat Min', 'trackbars')
                    s_max = cv2.getTrackbarPos('Sat Max', 'trackbars')
                    v_min = cv2.getTrackbarPos('Val Min', 'trackbars')
                    v_max = cv2.getTrackbarPos('Val Max', 'trackbars')

                    lower = np.array([h_min, s_min, v_min])
                    upper = np.array([h_max, s_max, v_max])
                    mask = cv2.inRange(hsv, lower, upper)
                    cv2.imshow('mask', mask)
                    cv2.imshow('frame', frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q') or key == ord('s') or key == ord('j'):
                        break
                
                
                if key == ord('q') or key == ord('j'):
                    break
            
            if key == ord('j'):
                print (value, lower, upper)
                self.hsv_dict[value] = [lower, upper]
                
        
        with open(self.BASE_DIR + '/hsv_colors.json', 'w') as f:

            # Serialize the dictionary and write it to the file, consider ndarray is not serializable
            hsv_json = {key: [value[0].tolist(), value[1].tolist()] for key, value in self.hsv_dict.items()}
            print (hsv_json)
            json.dump(hsv_json, f, indent=4)

        
def main(args=None):
    try:
        threshold_ = threshold()
        threshold_.getHSV()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()