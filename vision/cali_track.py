import cv2
import numpy as np

#Functions

#Function to select points in images, and obtain x and y image coordinates
def select_point(event, x, y, flags, param):
    global rig3D_pts
    global rig2D_pts
    global planeCalibration
    global selected_points
    if event == cv2.EVENT_LBUTTONDOWN:
        selected_points.append((x, y))
        arr = np.array([[x, y]], np.float32)
        if(planeCalibration):
            rig2D_pts = np.append(rig2D_pts, arr, axis=0)
            cv2.circle(img_rig2D, (x, y), 8, (0, 255, 0), -1) 
            cv2.imshow("2D rig", img_rig2D)
        else:
            rig3D_pts = np.append(rig3D_pts, arr, axis=0)
            cv2.circle(img_rig3D, (x, y), 8, (0, 0, 255), -1) 
            cv2.imshow("rig 3D", img_rig3D)


# Function to put several images in one window
def StackImages(scale, imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range (0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0,0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1],imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range (0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0,0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1],imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver

#Function to get the contours of an image after processing it.
def getContours(img, imgContour):
    #Obtain contours using the processed image
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2. CHAIN_APPROX_NONE)
    for cnt in contours:
        #Obtain area of each contour
        area = cv2.contourArea(cnt)
        if area > 1000:

            #Draw contours, obtain the perimeter and a approx Polygon
            cv2.drawContours(imgContour, cnt, -1, (255,0,255), 5)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, peri*0.02, True)

            #Obtain bounding rectangle that encloses the polygon, draw it and put a text beside it
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 5)
            cv2.putText(imgContour, "ID", (x + w + 20, y +20), cv2.FONT_HERSHEY_COMPLEX, .7, (0,255,0), 2)

# Function to process the image to do proper tracking
def imageProcessing(img):
    image_contour = img.copy()
    #Transform image to hsv
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    #Change values to get better mask
    h_min = cv2.getTrackbarPos("Hue min", "Trackbars")
    h_max = cv2.getTrackbarPos("Hue max", "Trackbars")
    s_min = cv2.getTrackbarPos("Sat min", "Trackbars")
    s_max = cv2.getTrackbarPos("Sat max", "Trackbars")
    v_min = cv2.getTrackbarPos("Val min", "Trackbars")
    v_max = cv2.getTrackbarPos("Val max", "Trackbars")

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    #Obtain mask of image
    mask = cv2.inRange(hsv, lower, upper)

    #Put mask on original image
    color_image = cv2.bitwise_and(img, img, mask=mask)

    #Blur image and change the color to gray scales
    imgBlur = cv2.GaussianBlur(color_image, (7,7), 1)
    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)

    # Use Canny Edge Detector
    threshold1 = cv2.getTrackbarPos("Threshold1", "Trackbars")
    threshold2 = cv2.getTrackbarPos("Threshold2", "Trackbars")

    imgCanny = cv2.Canny(imgGray, threshold1, threshold2)

    #Dilate the lines to facilitate getting contours
    kernel = np.ones((5,5))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)


    getContours(imgDil, image_contour)

    #Stack images one beside the other
    imgStack = StackImages(0.4, ([hsv, color_image, imgGray],
                                [imgCanny, imgDil, image_contour]))
    return imgStack

#--------------------------------------- CALIBRATION --------------------------------------------------------

#Cconstants

#Obtain points in 2D plane and 3D plane in centimeters
realp_2Dplane=np.array([[0,0,0],[0,5.4,0],[8.5,5.4,0],
                [8.5,0,0],[0.6,2.55,0],[0.6,3.55,0],
                [7.85,3.55,0],[7.85,2.55,0]], np.float32) 

realp_3Dplane=np.array([[8.5,0,5.4],[0,0,5.4],
                    [0,8.5,5.4],[0,8.5,0],
                    [0,5.4,0],[8.5,5.4,0],
                    [8.5,0,0],[0,0,0]], np.float32)

#Initiate 2D rig image points and 3D rig image points
rig2D_pts = np.zeros((0,2), np.float32)
rig3D_pts = np.zeros((0,2), np.float32)

#Initiate lists needed
obj_pts = []
img_pts = []
selected_points = []

#Read calibration images
img_rig2D = cv2.imread("Vision\Rig2D.jpg")
img_rig3D = cv2.imread("Vision\Rig3D.jpg")

planeCalibration = True

#Obtain image sizes
img_size_rig2D = img_rig2D.shape[:2]
img_size_rig3D = img_rig3D.shape[:2]

#Obtain 2D rig image points by selecting them in order
while True:
    cv2.imshow("2D rig", img_rig2D)
    cv2.setMouseCallback("2D rig", select_point)
    if cv2.waitKey(1) & 0xFF==ord('1'):
        break

cv2.destroyAllWindows()


obj_pts.append(realp_2Dplane)
img_pts.append(rig2D_pts)

# Estimation of Camera Matrix using 2D rig real points and image points
ret, CaliMtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, img_size_rig2D[::-1], None, None, None)

#Change calibration to 3D rig
planeCalibration = False

obj_pts.clear()
img_pts.clear()

#Obtain 3D rig image points by selecting them in order
while True:
    cv2.imshow("rig 3D", img_rig3D)
    # Set mouse callback function to handle clicks
    cv2.setMouseCallback("rig 3D", select_point)
    if cv2.waitKey(1) & 0xFF==ord('1'):
        break

obj_pts.append(realp_3Dplane)
img_pts.append(rig3D_pts)

#Second estimation of camera matrix with distorsion
ret, CaliMtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, img_size_rig3D[::-1], CaliMtx, None, None, None, flags=cv2.CALIB_USE_INTRINSIC_GUESS)

print("\nCamera matrix : \n", CaliMtx)
print("dist: \n", dist)
print("rvecs: ", rvecs)
print("tvecs: ", tvecs)

cv2.destroyAllWindows()


#--------------------------------- TRACKING ----------------------------------------

#Obtain video from webcam
frameWidth = 640
frameHeight = 480
cap = cv2.VideoCapture(1)
cap.set(3, frameWidth)
cap.set(4, frameHeight)

def empty(a):
    pass

#Create Trackbars
cv2.namedWindow("Trackbars")
cv2.resizeWindow("Trackbars", 640, 450)
cv2.createTrackbar("Threshold1", "Trackbars", 91, 255, empty)
cv2.createTrackbar("Threshold2", "Trackbars", 255, 255, empty)
cv2.createTrackbar("Hue min", "Trackbars", 81, 255, empty)
cv2.createTrackbar("Hue max", "Trackbars", 111, 255, empty)
cv2.createTrackbar("Sat min", "Trackbars", 103, 255, empty)
cv2.createTrackbar("Sat max", "Trackbars", 255, 255, empty)
cv2.createTrackbar("Val min", "Trackbars", 111, 255, empty)
cv2.createTrackbar("Val max", "Trackbars", 178, 255, empty)

#Show results
while True:
    success, img = cap.read()
    imgStack = imageProcessing(img)
    cv2.imshow("Results", imgStack)
    if cv2.waitKey(1) & 0xFF == ord('1'):
        break
