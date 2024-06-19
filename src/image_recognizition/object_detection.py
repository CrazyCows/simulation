import cv2
import imutils
import numpy as np
from cv2.gapi.wip.draw import Circle
from typing import List, Tuple
from src.dto.shapes import CircleObject, Position, SquareObject


def increase_vibrance(image, vibrance_scale=20, threshold_low=64, threshold_high=255):
    """
    Increase the vibrance of an image by selectively increasing its saturation with a smooth transition.

    Parameters:
    image (numpy.ndarray): Input image in BGR format.
    vibrance_scale (float): Scale factor for increasing saturation.
                            Values greater than 1.0 will increase saturation.
    threshold_low (int): Lower threshold for saturation values.
    threshold_high (int): Upper threshold for saturation values.

    Returns:
    numpy.ndarray: Image with increased vibrance.
    """
    # Convert the image from BGR to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Split the HSV image into its channels
    h, s, v = cv2.split(hsv_image)

    # Create a mask for the saturation increase
    mask_low = s >= threshold_low
    mask_high = s <= threshold_high

    # Create a smooth transition mask
    transition_mask = np.logical_and(mask_low, mask_high)
    scaling_factor = np.zeros_like(s, dtype=np.float32)

    scaling_factor[mask_low] = 1.0
    scaling_factor[transition_mask] = ((s[transition_mask] - threshold_low) / (threshold_high - threshold_low)) * (
            vibrance_scale - 1.0) + 1.0
    scaling_factor[mask_high] = vibrance_scale

    # Apply the scaling factor to the saturation channel
    s = np.clip(s * scaling_factor, 0, 255).astype(np.uint8)

    # Merge the channels back
    hsv_image = cv2.merge([h, s, v])

    # Convert the image back from HSV to BGR color space
    final_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)

    return final_image


def calculate_positive_angle(circle1: CircleObject, circle2: CircleObject) -> float:
    # IDK if its necessary to convert to np arrays
    pos1 = np.array([circle1.position.x, circle1.position.y])
    pos2 = np.array([circle2.position.x, circle2.position.y])
    delta = pos2 - pos1
    # These are switched per "(Note the role reversal: the “y-coordinate” is the first function parameter, the “x-coordinate” is the second."
    # - numpy docs
    angle = np.arctan2(delta[1], delta[0])
    angle = angle - np.pi / 2 - np.pi
    angle = -angle
    if angle < 0:
        angle += 2 * np.pi
    elif angle > 2 * np.pi:
        angle = angle - 2 * np.pi
    return angle


class RoboVision:
    # at a camera height of 202cm with
    _whiteSizeLower = 7
    _whiteSizeUpper = 12
    _eggSizeLower = _whiteSizeUpper + 1
    _eggSizeUpper = 50
    _dotSizeLower = 1
    _dotSizeUpper = 70
    _robot_width = 100
    _robot_height = 100

    _whiteLower = np.array([0, 0, 220])
    _whiteUpper = np.array([255, 100, 255])

    _red1lower_limit = np.array([0, 125, 80])
    _red1upper_limit = np.array([15, 255, 255])

    _red2lower_limit = np.array([160, 125, 80])
    _red2upper_limit = np.array([179, 255, 255])

    _green_lower_limit = np.array([30, 180, 45])
    _green_upper_limit = np.array([90, 255, 255])
    # Id like to avoid overlap in these filters
    # IS SET TO BLACC
    _blue_lower_limit = np.array([90, 180, 60])
    _blue_upper_limit = np.array([130, 255, 255])

    _orange_lower_limit = np.array([15, 240, 140])
    _orange_upper_limit = np.array([50, 255, 255])

    """ Outdated values
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
    """

    _cross_area = 100
    _vs = cv2.VideoCapture(0)
    # Set the resolution to 1920x1080
    _vs.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    _vs.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # Variables to be updated every darn iteration
    _blurred = None
    _hsv = None

    def commonSetup(self):
        # Yes, this has to be here twice.
        # Very first frame from droidcam is just orange for
        # some reason
        ret, frame = self._vs.read()
        ret, frame = self._vs.read()

        if frame is None:
            raise Exception("Camera error")
        frame = increase_vibrance(frame, 1.5)
        self._blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        self._hsv = cv2.cvtColor(self._blurred, cv2.COLOR_BGR2HSV)

    def _getBallishThing(self, lowerMask, upperMask, lowerSize, upperSize) -> List[CircleObject]:
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

    def _get_cross(self):
        self.commonSetup()
        redLowerMask = cv2.inRange(self._hsv, self._red1lower_limit, self._red1upper_limit)
        redUpperMask = cv2.inRange(self._hsv, self._red2lower_limit, self._red2upper_limit)
        redFullMask = redLowerMask + redUpperMask
        redFullMask = cv2.erode(redFullMask, None, iterations=2)
        redFullMask = cv2.dilate(redFullMask, None, iterations=2)
        cnts, _ = cv2.findContours(redFullMask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        approximations = []
        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if area > self._cross_area:
                _, radius = cv2.minEnclosingCircle(cnt)
                epsilon = 0.0125 * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True, cv2.CHAIN_APPROX_SIMPLE)
                approximations.append(approx)
        print(approximations)
        for approx in approximations:
            for thing in approx:
                print("x: " + str(thing[0][0]))
                print("y: " + str(thing[0][1]))

        new_approx = []
        for approx in approximations:
            pass

        return approximations

    def _get_robot_center(self) -> Tuple[CircleObject, float]:
        greendots = []
        while len(greendots) == 0:
            print("Looking for green dots")
            greendots = self._getBallishThing(self._green_lower_limit, self._green_upper_limit, self._dotSizeLower,
                                              self._dotSizeUpper)
        greendot = greendots[0]
        bluedots = []
        while len(bluedots) == 0:
            print("Looking for blue dots")
            bluedots = self._getBallishThing(self._blue_lower_limit, self._blue_upper_limit, self._dotSizeLower,
                                             self._dotSizeUpper)
        bluedot = bluedots[0]
        center = CircleObject(radius=1,
                              position=Position(x=int((greendot.position.x + bluedot.position.x) / 2),
                                                y=int((greendot.position.y + bluedot.position.y) / 2)))
        angle = calculate_positive_angle(greendot, bluedot)
        return center, angle

    def _get_white_balls(self) -> List[CircleObject]:
        return self._getBallishThing(self._whiteLower, self._whiteUpper, self._whiteSizeLower, self._whiteSizeUpper)

    def _get_orange_ball(self) -> List[CircleObject]:
        orange_balls = self._getBallishThing(self._orange_lower_limit, self.orange, self._whiteSizeLower,
                                             self._whiteSizeUpper)
        return orange_balls

    def _get_egg(self) -> List[CircleObject]:
        return self._getBallishThing(self._whiteLower, self._whiteUpper, self._eggSizeLower, self._eggSizeUpper)

    def _get_robot_square(self) -> SquareObject:
        circle, angle = self._get_robot_center()

        square = SquareObject.create_square(circle.position,
                                            self._robot_width,
                                            self._robot_height,
                                            angle
                                            )

        return square

    def get_any_thing(self, min_count=0, max_count=100000, tries=25, thing_to_get=""):

        if thing_to_get == "white_ball":
            func = self._get_white_balls
        elif thing_to_get == "orange_ball":
            func = self._get_orange_ball
        elif thing_to_get == "egg":
            func = self._get_egg
        elif thing_to_get == "cross":
            func = self._get_cross
        elif thing_to_get == "robot":
            func = self._get_robot_square
        else:
            raise Exception("Invalid argument")
        if thing_to_get != "robot":
            for _ in range(tries):
                list_of_thing = func()
                if min_count <= len(list_of_thing) <= max_count:
                    return list_of_thing
            print(thing_to_get + " not found within parameters")
        else:
            robot = func()
            while not robot:
                robot = func
            return robot


if __name__ == '__main__':
    pass
