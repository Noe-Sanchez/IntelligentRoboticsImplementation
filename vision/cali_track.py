import cv2
import numpy as np

#-------------- CALIBRACION ------------------
#Constantes
realp_2Dplane=np.array([[0,0,0],[0,5.4,0],[8.5,5.4,0],
                [8.5,0,0],[0.6,2.55,0],[0.6,3.55,0],
                [7.85,3.55,0],[7.85,2.55,0]], np.float32) 

realp_3Dplane=np.array([[8.5,0,5.4],[0,0,5.4],
                    [0,8.5,5.4],[0,8.5,0],
                    [0,5.4,0],[8.5,5.4,0],
                    [8.5,0,0],[0,0,0]], np.float32)

rig2D_pts = np.zeros((0,2), np.float32)
rig3D_pts = np.zeros((0,2), np.float32)

obj_pts = []
img_pts = []
selected_points = []

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

  
img_rig2D = cv2.imread("Vision\Rig2D.jpg")
img_rig3D = cv2.imread("Vision\Rig3D.jpg")

planeCalibration = True

img_size_rig2D = img_rig2D.shape[:2]
img_size_rig3D = img_rig3D.shape[:2]

while True:
    cv2.imshow("2D rig", img_rig2D)
    # Set mouse callback function to handle clicks
    cv2.setMouseCallback("2D rig", select_point)
    if cv2.waitKey(1) & 0xFF==ord('1'):
        break

cv2.destroyAllWindows()


obj_pts.append(realp_2Dplane)
img_pts.append(rig2D_pts)

# Estimation of Camera Matrix
ret, CaliMtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, img_size_rig2D[::-1], None, None, None)

planeCalibration = False

obj_pts.clear()
img_pts.clear()

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


#----------- TRACKING --------------------

frameWidth = 640
frameHeight = 480
cap = cv2.VideoCapture(1)
cap.set(3, frameWidth)
cap.set(4, frameHeight)

def empty(a):
    pass

cv2.namedWindow("Trackbars")
cv2.resizeWindow("Trackbars", 640, 450)
cv2.createTrackbar("Threshold1", "Trackbars", 161, 255, empty)
cv2.createTrackbar("Threshold2", "Trackbars", 171, 255, empty)
cv2.createTrackbar("Hue min", "Trackbars", 95, 255, empty)
cv2.createTrackbar("Hue max", "Trackbars", 133, 255, empty)
cv2.createTrackbar("Sat min", "Trackbars", 32, 255, empty)
cv2.createTrackbar("Sat max", "Trackbars", 255, 255, empty)
cv2.createTrackbar("Val min", "Trackbars", 124, 255, empty)
cv2.createTrackbar("Val max", "Trackbars", 255, 255, empty)


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


def getContours(img, imgContour):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2. CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 5000:
            cv2.drawContours(imgContour, cnt, -1, (255,0,255), 5)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, peri*0.02, True)
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 5)
            cv2.putText(imgContour, "ID", (x + w + 20, y +20), cv2.FONT_HERSHEY_COMPLEX, .7, (0,255,0), 2)


def imageProcessing(img):
    image_contour = img.copy()
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("Hue min", "Trackbars")
    h_max = cv2.getTrackbarPos("Hue max", "Trackbars")
    s_min = cv2.getTrackbarPos("Sat min", "Trackbars")
    s_max = cv2.getTrackbarPos("Sat max", "Trackbars")
    v_min = cv2.getTrackbarPos("Val min", "Trackbars")
    v_max = cv2.getTrackbarPos("Val max", "Trackbars")

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    mask = cv2.inRange(hsv, lower, upper)

    color_image = cv2.bitwise_and(img, img, mask=mask)

    imgBlur = cv2.GaussianBlur(color_image, (7,7), 1)
    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)

    threshold1 = cv2.getTrackbarPos("Threshold1", "Trackbars")
    threshold2 = cv2.getTrackbarPos("Threshold2", "Trackbars")

    imgCanny = cv2.Canny(imgGray, threshold1, threshold2)
    kernel = np.ones((5,5))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)
    getContours(imgDil, image_contour)
    imgStack = StackImages(0.4, ([hsv, color_image, imgGray],
                                [imgCanny, imgDil, image_contour]))
    return imgStack

while True:
    success, img = cap.read()
    imgStack = imageProcessing(img)
    cv2.imshow("Results", imgStack)
    if cv2.waitKey(1) & 0xFF == ord('1'):
        break
