import cv2
import numpy as np
import glob
import os
import pathlib

BASE_DIR = str(pathlib.Path(__file__).resolve().parent)

def getPics(calibrationDir):
    cap = cv2.VideoCapture(0)
    cnt = 0
    while True:
        ret, frame = cap.read()
        if not ret or cnt >= 10:
            break
        cv2.imshow('camera', frame)
        img_name = os.path.join(calibrationDir, f'calibrationImg_{cnt}.jpg')
        if cv2.waitKey(1) & 0xFF==ord('1'):
            cv2.imwrite(img_name, frame)
            cv2.imshow('Captured img', frame)
            cnt += 1
    cap.release()
    cv2.destroyAllWindows()

def calibrate(showPics=True, takePic=False):
    calibrationDir = os.path.join(BASE_DIR,'calibrationImg')
    
    if takePic:
        getPics(calibrationDir)

    imgPathList = glob.glob(os.path.join(calibrationDir, 'calibrationImg_*.jpg'))
    print("Calibrating with {} images".format(len(imgPathList)))
    nRows = 7
    nCols = 7
    square_size = 0.7 #cm
    termCriteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    worldPtsCur = np.zeros((nRows*nCols, 3), np.float32)
    worldPtsCur[:,:2] = np.mgrid[0:nRows, 0:nCols].T.reshape(-1,2)
    worldPtsCur = worldPtsCur*square_size
    worldPtsList = []
    imgPtsList = []

    # create hsv bounds with the following values hMin = 100 , sMin = 20, vMin = 135), (hMax = 135 , sMax = 100, vMax = 190)
    lower = np.array([100, 20, 135])
    upper = np.array([135, 100, 190])

    for curImgPath in imgPathList:
        imgBGR = cv2.imread(curImgPath)
        imgGrey = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2GRAY)
        imgHSV = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(imgHSV, lower, upper)
        cv2.imshow('Mask', mask)
        krn = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 30))
        dlt = cv2.dilate(mask, krn, iterations=5)
        res = 255 - cv2.bitwise_and(dlt, mask)

        res = np.uint8(res)

        cornersFound, cornersOrg = cv2.findChessboardCorners(res, (nRows,nCols), None)
        cv2.waitKey(500)
        cv2.imshow('Original', res)

        cv2.imshow('Grey', imgGrey)

        if not cornersFound:
            cornersFound, cornersOrg = cv2.findChessboardCorners(imgGrey, (nRows,nCols), None)
            
        if cornersFound:
            print ('Found corners in ', curImgPath)
            worldPtsList.append(worldPtsCur)
            cornersRefined = cv2.cornerSubPix(res, cornersOrg, (11,11), (-1,-1), termCriteria)
            imgPtsList.append(cornersRefined)
            if showPics:
                cv2.drawChessboardCorners(imgBGR, (nRows, nCols), cornersRefined, cornersFound)
                cv2.imshow('Chessboard', imgBGR)
                cv2.waitKey(500)
    cv2.destroyAllWindows()

    repError, camMatrix, distCoeff, rvecs, tvecs = cv2.calibrateCamera(worldPtsList, imgPtsList, res.shape[::-1], None, None)
    print('Camera matrix: \n', camMatrix)
    print('reproj error (px): {:.4f}'.format(repError))

    
    paramPath = os.path.join(BASE_DIR, 'calibration.npz')
    np.savez(paramPath,
             repError=repError,
             camMatrix=camMatrix,
             distCoeff=distCoeff,
             rvecs=rvecs,
             tvecs=tvecs)