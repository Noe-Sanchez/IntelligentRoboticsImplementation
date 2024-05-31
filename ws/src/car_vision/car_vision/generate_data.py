import cv2
import os
import glob
import pathlib
import numpy as np
import json

import time 

class dataGeneration():
    def __init__(self):
        self.BASE_DIR = str(pathlib.Path(__file__).resolve().parent)
        self.saving_dir = self.BASE_DIR + '/saved_images'
        self.dataset_dir = self.BASE_DIR + '/dataset'
        self.frame = None
        self.flag = False
        self.cnt = 0

        self.current_label = ord('0')
        self.labels = self.getLabelsDict()

        self.color_dict = {
            "forward" : "blue",
            "turn_left" : "blue",
            "turn_right" : "blue",
            "roundabout" : "blue",
            "stop" : "red",
            "giveaway" : "red",
            "roadwork" : "red"
        }

        self.showDict()

        # create the directories for pngs 
        for key, value in self.labels.items():
            if not os.path.exists(self.dataset_dir + f'/{value}_png'):
                os.makedirs(self.dataset_dir + f'/{value}_png')

    def showDict(self):
        # Show labels as with its key number
        for key, value in self.labels.items():
            print(f'{chr(key)}: {value}')
        
    
    def getLabelsDict(self):
        labels = {}
        cnt = 0
        for i, file in enumerate(glob.glob(self.dataset_dir + '/*')):
            print(file)
            print(cnt)
            if file.find('png') != -1:
                continue
            labels[ord(str(cnt + 1))] = file.removeprefix(self.dataset_dir + '/')
            cnt += 1
        return labels
    
    def getHSV(self, reset=False):
        # Get the HSV values for the colors from a json file
        try:
            with open(self.BASE_DIR + '/hsv.json', 'r') as f:
                self.hsv_dict = json.load(f) 
        except FileNotFoundError:
            self.hsv_dict = {
                "blue": [[0, 0, 0], [255, 255, 255]],
                "red": [[0, 0, 0], [255, 255, 255]],
            }

        # Deserialize the dictionary and convert the values to numpy array
        self.hsv_dict = {key: [np.array(value[0]), np.array(value[1])] for key, value in self.hsv_dict.items()}

        if not reset:
            return

        # Mannualy change the values using trackbars once for each color
        cv2.namedWindow('trackbars')
        cv2.createTrackbar('Hue Min', 'trackbars', 0, 255, lambda x: None)
        cv2.createTrackbar('Hue Max', 'trackbars', 0, 255, lambda x: None)
        cv2.createTrackbar('Sat Min', 'trackbars', 0, 255, lambda x: None)
        cv2.createTrackbar('Sat Max', 'trackbars', 0, 255, lambda x: None)
        cv2.createTrackbar('Val Min', 'trackbars', 0, 255, lambda x: None)
        cv2.createTrackbar('Val Max', 'trackbars', 0, 255, lambda x: None)

        for key, value in self.labels.items():
            cv2.setTrackbarPos('Hue Min', 'trackbars', self.hsv_dict[self.color_dict[value]][0][0])
            cv2.setTrackbarPos('Hue Max', 'trackbars', self.hsv_dict[self.color_dict[value]][1][0])
            cv2.setTrackbarPos('Sat Min', 'trackbars', self.hsv_dict[self.color_dict[value]][0][1])
            cv2.setTrackbarPos('Sat Max', 'trackbars', self.hsv_dict[self.color_dict[value]][1][1])
            cv2.setTrackbarPos('Val Min', 'trackbars', self.hsv_dict[self.color_dict[value]][0][2])
            cv2.setTrackbarPos('Val Max', 'trackbars', self.hsv_dict[self.color_dict[value]][1][2])
            for file in glob.glob(self.dataset_dir + f'/{value}/*'):
                frame = cv2.imread(file)
                

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
                self.hsv_dict[self.color_dict[value]] = [lower, upper]
        
        with open(self.BASE_DIR + '/hsv.json', 'w') as f:

            # Serialize the dictionary and write it to the file, consider ndarray is not serializable
            hsv_json = {key: [value[0].tolist(), value[1].tolist()] for key, value in self.hsv_dict.items()}
            json.dump(hsv_json, f, indent=4)


    def getTrainingData(self):
        # Get the training data
        self.cnt = 0
        for key, value in self.labels.items():
            for file in glob.glob(self.dataset_dir + f'/{value}/*'):
                img = cv2.imread(file)
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, self.hsv_dict[self.color_dict[value]][0], self.hsv_dict[self.color_dict[value]][1])

                # dilate the image
                kernel = np.ones((1, 1), np.uint8)
                mask = cv2.erode(mask, kernel, iterations=7)

                # dilate the image
                kernel = np.ones((7, 7), np.uint8)
                mask = cv2.dilate(mask, kernel, iterations=3)

                cv2.imshow('mask', mask)

                # Get bounding box using contours
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # get the largest contour
                if len(contours) == 0:
                    continue
                
                
                c = max(contours, key=cv2.contourArea)

                
                modifiable = img.copy() 
                fill = img.copy()

                # draw the countour
                cv2.drawContours(modifiable, [c], -1, (0, 255, 0), 2)
                cv2.drawContours(fill, [c], -1, (255, 255, 255), -1)

                cv2.imshow('filled', fill)

                x, y, w, h = cv2.boundingRect(c)

                # Draw the bounding box with padding
                cv2.rectangle(modifiable, (x - 10, y - 10), (x + w + 10, y + h + 10), (0, 255, 0), 2)
                cv2.imshow('modifiable', modifiable)

                #Crop the image

                crop_img = img[y - 10:y + h + 10, x - 10:x + w + 10]
                crop_fill = fill[y - 10:y + h + 10, x - 10:x + w + 10]
                
                if crop_img.size == 0:
                    crop_img = img[y:y + h, x:x + w]
                if crop_fill.size == 0:
                    crop_fill = fill[y:y + h, x:x + w]

                cv2.imshow('crop_img', crop_img)

                # convert to binary the crop filled image
                crop_fill_mod = cv2.cvtColor(crop_fill, cv2.COLOR_BGR2GRAY)
                _, crop_fill_mod = cv2.threshold(crop_fill_mod, 254, 255, cv2.THRESH_BINARY)

                # dialate the image
                kernel = np.ones((5, 5), np.uint8)
                crop_fill_mod = cv2.dilate(crop_fill_mod, kernel, iterations=2)

                #erode
                crop_fill_mod = cv2.erode(crop_fill_mod, kernel, iterations=2)
                cv2.imshow('crop_fill_mod', crop_fill_mod)

                # convert the image to png and put alpha value on cero for the black pixels
                crop_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2BGRA)
                crop_img[:, :, 3] = crop_fill_mod

                cv2.imshow('crop_png', crop_img)

                # save the image
                cv2.imwrite(self.dataset_dir + f'/{value}_png/{self.cnt}.png', crop_img)
                self.cnt += 1
                if cv2.waitKey(100) & 0xFF == ord('q'):
                    break
            self.cnt = 0




        
def main(args=None):
    try:
        dataGeneration_ = dataGeneration()
        dataGeneration_.getHSV()
        dataGeneration_.getTrainingData()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()