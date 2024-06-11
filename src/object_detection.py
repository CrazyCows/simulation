import cv2
import imutils
import numpy as np
from src.dto import CircleObject, SquareObject, Position

class RoboVision:
    _whiteLower = np.array([0, 0, 220])
    _whiteUpper = np.array([255, 100, 255])
    _whiteSizeLower = 5
    _whiteSizeUpper = 20
    _eggSizeLower = _whiteSizeUpper
    _eggSizeUpper = 50

    _red1lower_limit = np.array([0, 100, 20])
    _red1upper_limit = np.array([10, 255, 255])
    _red2lower_limit = np.array([160, 40, 20])
    _red2upper_limit = np.array([179, 255, 255])

    _cross_area = 100
    _vs = cv2.VideoCapture(1)


    #Variables to be updated every darn iteration
    _blurred = None
    _hsv = None

    def commonSetup(self):
        #Yes, this has to be here twice.
        #Very first frame from droidcam is just orange for
        #some reason
        ret, frame = self._vs.read()
        ret, frame = self._vs.read()

        if frame is None:
            raise Exception("Camera error")
        self._blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        self._hsv = cv2.cvtColor(self._blurred, cv2.COLOR_BGR2HSV)

    def _getBallishThing(self, lowerMask, upperMask, lowerSize, upperSize):
        self.commonSetup()
        mask = cv2.inRange(self._hsv, lowerMask, upperMask)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        balls = []

        for cnt in cnts:
            ((x, y), radius) = cv2.minEnclosingCircle(cnt)
            if lowerSize < radius < upperSize:
                balls.append(CircleObject(radius=int(radius), position=Position(x=x, y=y)))
        return balls
    
    def getCross(self):
        self.commonSetup()
        redLowerMask = cv2.inRange(self._hsv, self._red1lower_limit, self._red1upper_limit)
        redUpperMask = cv2.inRange(self._hsv, self._red2lower_limit, self._red2upper_limit)
        redFullMask = redLowerMask + redUpperMask
        redFullMask = cv2.erode(redFullMask, None, iterations=2)
        redFullMask = cv2.dilate(redFullMask, None, iterations=2)
        cnts, _ = cv2.findContours(redFullMask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        _approximations = []
        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if area > self._cross_area:
                _, radius = cv2.minEnclosingCircle(cnt)
                epsilon = 0.01 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True, cv2.CHAIN_APPROX_SIMPLE)
                _approximations.append(approx)
        return _approximations



    def getWhiteBalls(self):
        return self._getBallishThing(self._whiteLower, self._whiteUpper, self._whiteSizeLower, self._whiteSizeUpper)

    def getOrangeBall(self):
        return self._getBallishThing(self.todo, self.todo, self._whiteSizeLower, self._whiteSizeUpper)

    def getEgg(self):
        return self._getBallishThing(self._whiteLower, self._whiteUpper, self._eggSizeLower, self._eggSizeUpper)

    #Returns a list of approximated poly DP, see https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html#ga0012a5fdaea70b8a9970165d98722b4c






if __name__ == '__main__':
    robo = RoboVision()
    whiteBalls = robo.getWhiteBalls()
    #orangeBalls = robo.getOrangeBall()
    cross = robo.getCross()

    for item in whiteBalls:
        print(item)
