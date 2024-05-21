import cv2
import numpy as np
import glob
import os

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
    root = os.getcwd()
    calibrationDir = os.path.join(root,'calibrationImg')
    
    if takePic:
        getPics(calibrationDir)

    imgPathList = glob.glob(os.path.join(calibrationDir, 'calibrationImg_*.jpg'))

    nRows = 8
    nCols = 4
    square_size = 0.9 #cm
    termCriteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    worldPtsCur = np.zeros((nRows*nCols, 3), np.float32)
    worldPtsCur[:,:2] = np.mgrid[0:nRows, 0:nCols].T.reshape(-1,2)
    worldPtsCur = worldPtsCur*square_size
    worldPtsList = []
    imgPtsList = []
    for curImgPath in imgPathList:
        imgBGR = cv2.imread(curImgPath)
        imgGray = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2GRAY)
        cornersFound, cornersOrg = cv2.findChessboardCorners(imgGray, (nRows,nCols), None)
        cv2.waitKey(500)
        if cornersFound:
            worldPtsList.append(worldPtsCur)
            cornersRefined = cv2.cornerSubPix(imgGray, cornersOrg, (11,11), (-1,-1), termCriteria)
            imgPtsList.append(cornersRefined)
            if showPics:
                cv2.drawChessboardCorners(imgBGR, (nRows, nCols), cornersRefined, cornersFound)
                cv2.imshow('Chessboard', imgBGR)
                cv2.waitKey(500)
    cv2.destroyAllWindows()

    repError, camMatrix, distCoeff, rvecs, tvecs = cv2.calibrateCamera(worldPtsList, imgPtsList, imgGray.shape[::-1], None, None)
    print('Camera matrix: \n', camMatrix)
    print('reproj error (px): {:.4f}'.format(repError))

    curFolder = os.path.dirname(os.path.abspath(__file__))
    paramPath = os.path.join(curFolder, 'calibration.npz')
    np.savez(paramPath,
             repError=repError,
             camMatrix=camMatrix,
             distCoeff=distCoeff,
             rvecs=rvecs,
             tvecs=tvecs)

if __name__ == '__main__':
    calibrate(takePic=False)
