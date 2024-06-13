import cv2
import imutils
import numpy as np
from cv2.gapi.wip.draw import Circle

from src.dto import CircleObject, SquareObject, Position


def calculate_positive_angle(circle1: CircleObject, circle2: CircleObject) -> float:
    pos1 = np.array([circle1.position.x, circle1.position.y])
    pos2 = np.array([circle2.position.x, circle2.position.y])
    delta = pos2 - pos1
    angle = np.arctan2(delta[1], delta[0])
    if angle < 0:
        angle += 2 * np.pi
    return angle


class RoboVision:
    _whiteLower = np.array([0, 0, 220])
    _whiteUpper = np.array([255, 100, 255])
    _whiteSizeLower = 5
    _whiteSizeUpper = 20
    _eggSizeLower = _whiteSizeUpper
    _eggSizeUpper = 50
    _dotSizeLower = 2
    _dotSizeUpper = 1000000

    #Outdated values
    """
    _red1lower_limit = np.array([0, 100, 20])
    _red1upper_limit = np.array([10, 255, 255])
    _red2lower_limit = np.array([160, 40, 20]) #TODO: Sat here should be higher ideally
    _red2upper_limit = np.array([179, 255, 255])

    #_green_lower_limit = np.array([67, 70, 40])
    #_green_upper_limit = np.array([97, 255, 255])
    #_blue_lower_limit = np.array([98, 70, 40])
    #_blue_upper_limit = np.array([145, 255, 255])
    _green_lower_limit = np.array([55, 100, 70])
    _green_upper_limit = np.array([80, 255, 255])
    # Id like to avoid overlap in these filters
    _blue_lower_limit = np.array([95, 180, 40])
    _blue_upper_limit = np.array([115, 255, 255])
    """

    whiteLower = np.array([0, 0, 220])
    whiteUpper = np.array([255, 100, 255])

    #Theres two of these because red is where the circle of HSV "overlaps"
    red1lower_limit = np.array([0, 125, 20])
    red1upper_limit = np.array([10, 255, 255])
    red2lower_limit = np.array([160, 125, 20])
    red2upper_limit = np.array([179, 255, 255])

    _green_lower_limit = np.array([55, 100, 70])
    _green_upper_limit = np.array([80, 255, 255])
    # Id like to avoid overlap in these filters
    _blue_lower_limit = np.array([95, 180, 40])
    _blue_upper_limit = np.array([115, 255, 255])

    _orange_lower_limit = np.array([10, 100, 40])
    _orange_upper_limit = np.array([25, 255, 255])

    _cross_area = 100
    _vs = cv2.VideoCapture(0)


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



    def get_white_balls(self):
        return self._getBallishThing(self._whiteLower, self._whiteUpper, self._whiteSizeLower, self._whiteSizeUpper)

    def get_orange_ball(self):
        return self._getBallishThing(self.todo, self.todo, self._whiteSizeLower, self._whiteSizeUpper)

    def get_egg(self):
        return self._getBallishThing(self._whiteLower, self._whiteUpper, self._eggSizeLower, self._eggSizeUpper)

    def get_robot(self):
        greendots = self._getBallishThing(self._green_lower_limit, self._green_upper_limit, self._dotSizeLower, self._dotSizeUpper)
        if len(greendots) == 0:
            raise Exception("No green dots detected")
        elif len(greendots) > 1:
            raise Exception("Multiple green dots detected")
        greendot = greendots[0]
        bluedots = self._getBallishThing(self._blue_lower_limit, self._blue_upper_limit, self._dotSizeLower, self._dotSizeUpper)
        if len(bluedots) == 0:
            raise Exception("No blue dots detected")
        elif len(bluedots) > 1:
            raise Exception("Multiple blue dots detected")
        bluedot = bluedots[0]
        center = CircleObject(radius=1,
                              position=Position(x=int((greendot.position.x+bluedot.position.x)/2),
                                                y=int((greendot.position.y+bluedot.position.y)/2)))
        angle = calculate_positive_angle(greendot, bluedot)
        print("Center: " + str(center) + "  Angle: " + str(angle))
        return center, angle




if __name__ == '__main__':

    robo = RoboVision()
    while True:
        try:
            robo.get_robot()
        except:
            continue
    whiteBalls = robo.get_white_balls()
    #orangeBalls = robo.getOrangeBall()
    cross = robo.getCross()

