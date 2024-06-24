import math

import cv2
import imutils
import numpy as np
from cv2.gapi.wip.draw import Circle
from typing import List, Tuple
from dto.shapes import CircleObject, Position, SquareObject
import torch
from ultralytics import YOLO

class NoRobotException(Exception):
    "Raised when a robot is not found"
    pass


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


class RoboVision():
    # at a camera height of 202cm with
    _whiteSizeLower = 7
    _whiteSizeUpper = 12
    _eggSizeLower = _whiteSizeUpper + 1
    _eggSizeUpper = 50
    _dotSizeLower = 5
    _dotSizeUpper = 60
    _min_x = 1000000
    _max_x = 0
    _min_y = 1000000
    _max_y = 0

    def __init__(self, walls: List[SquareObject], ai: bool = False, power: int = 1):
        for wall in walls:
            #print("")
            for vertex in wall.vertices:  # Not very pythonic
                if vertex[0] > self._max_x:
                    self._max_x = vertex[0]
                if vertex[0] < self._min_x:
                    self._min_x = vertex[0]
                if vertex[1] > self._max_y:
                    self._max_y = vertex[1]
                if vertex[1] < self._min_y:
                    self._min_y = vertex[1]
        print("Minimum wall x position: " + str(self._min_x))
        print("Maximum wall x position: " + str(self._max_x))
        print("Minimum wall y position: " + str(self._min_y))
        print("Maximum wall y position: " + str(self._max_y))

        self.last_robot_square = SquareObject

        self.ai: bool = ai

        if self.ai:
            self.model = self.load_yolo_model(power=power)

        self._frame = None
        self.orientation: float = 0.0

    _robot_y = 100
    _robot_x = 100
    _robot_z_cm = 32
    _camera_z_cm = 190
    _camera_x: int = None
    _camera_y: int = None
    _z_factor = 1 - (_camera_z_cm - _robot_z_cm) / _camera_z_cm

    _whiteLower = np.array([0, 0, 220])
    _whiteUpper = np.array([255, 50, 255])

    _red1lower_limit = np.array([0, 125, 80])
    _red1upper_limit = np.array([15, 255, 255])

    _red2lower_limit = np.array([160, 125, 80])
    _red2upper_limit = np.array([179, 255, 255])

    _green_lower_limit = np.array([50, 30, 45])
    _green_upper_limit = np.array([90, 255, 255])
    # Id like to avoid overlap in these filters
    # IS SET TO BLACC
    _blue_lower_limit = np.array([100, 170, 80])
    _blue_upper_limit = np.array([125, 255, 255])

    _orange_lower_limit = np.array([15, 250, 235])
    _orange_upper_limit = np.array([32, 255, 255])

    # _orange_lower_limit = np.array([15, 240, 140])
    # _orange_upper_limit = np.array([50, 255, 255])

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
    _vs = cv2.VideoCapture(1)
    _vs.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    _vs.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # Variables to be updated every darn iteration
    _blurred = None
    _hsv = None
    _last_frame = None

    def load_yolo_model(self, power: int):
        if torch.cuda.is_available():
            print(f"CUDA is available. Version: {torch.version.cuda}")
            device = torch.device('cuda')
            print(f"Using device: {device}")
            print(f"GPU: {torch.cuda.get_device_name(0)}")
        else:
            device = torch.device('cpu')
            print("CUDA is not available. Using CPU.")

        # How powerful the model running on the computer should be
        # Light
        if (power == 1):
            model = YOLO(r"C:\Users\LuucM\PycharmProjects\simulation\src\image_recognizition\models\light.pt").to(device)
        # Medium
        elif power == 2:
            model = YOLO(r"C:\Users\LuucM\PycharmProjects\simulation\src\image_recognizition\models\medium.pt").to(device)
        # Heavy ()
        elif power == 3:
            model = YOLO(r"C:\Users\LuucM\PycharmProjects\simulation\src\image_recognizition\models\heavy.pt").to(device)
        return model


    def get_flipped_frame(self):
        ret, frame = self._vs.read()
        cv2.flip(frame, 1, frame)
        return frame

    """
    def get_flipped_frame(self):
        new_frame = self._last_frame.copy()
        cv2.flip(new_frame, 1, new_frame)
        return new_frame
    
    """

    def commonSetup(self):
        ret, frame = self._vs.read()

        if frame is None:
            raise Exception("Camera error")
        self._last_frame = frame
        # print(type(frame))
        # print("Dimensions:" + str(frame.shape))
        # frame = frame[int(self._min_y):int(self._max_y), int(self._min_x):int(self._max_x)]
        # print(type(frame))
        # print("Dimensions:" + str(frame.shape))

        if self._camera_x is None:
            self._camera_x = int(self._vs.get(cv2.CAP_PROP_FRAME_WIDTH) / 2)
            self._camera_y = int(self._vs.get(cv2.CAP_PROP_FRAME_HEIGHT) / 2)

        #self._blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        self._frame = frame
        self._hsv = frame

    def _getBallishThing(self, lowerMask, upperMask, lowerSize, upperSize, decrease_tolerance=False) -> List[
        CircleObject]:
        self.commonSetup()
        mask = cv2.inRange(self._hsv, lowerMask, upperMask)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        balls = []

        for cnt in cnts:
            ((x, y), radius) = cv2.minEnclosingCircle(cnt)
            if lowerSize < radius < upperSize and self._max_x > x > self._min_x and self._max_y > y > self._min_y:
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
        #print(approximations)
        for approx in approximations:
            for thing in approx:
                ""
                #print("x: " + str(thing[0][0]))
                #print("y: " + str(thing[0][1]))

        return approximations

    def _correct_robot_location_perspective(self, before_center: CircleObject) -> CircleObject:
        before_center.position.x = before_center.position.x + self._z_factor * (
                self._camera_x - before_center.position.x)
        before_center.position.y = before_center.position.y + self._z_factor * (
                self._camera_y - before_center.position.y)
        return before_center

    def _correct_point_location_perspective(self, before_center: CircleObject) -> CircleObject:
        before_center.position.x = before_center.position.x + self._z_factor * (
                self._camera_x - before_center.position.x)
        before_center.position.y = before_center.position.y + self._z_factor * (
                self._camera_y - before_center.position.y)
        return before_center

    def _get_robot_center(self) -> Tuple[CircleObject, float]:
        green_dots = []
        blue_dots = []
        # TODO: These loops should probably just be removed. I dont want to retry extensively here,
        # Because retrying 30 times (one second) could cause significant desync between the two dots,
        # leading to a misrepresented location
        for i in range(2):
            if self.ai:
                blue_dots = self.detect_with_yolo("purple_back")
            else:
                blue_dots = self._getBallishThing(self._blue_lower_limit, self._blue_upper_limit, self._dotSizeLower,
                                                  self._dotSizeUpper)

            # print("Looking for blue dot. Current number of blue dots: " + str(len(blue_dots)))
            if len(blue_dots) == 1:
                break
        if len(blue_dots) > 1:
            raise NoRobotException("More than one blue dot")
        elif len(blue_dots) == 0:
            raise NoRobotException("No blue dots")
        for i in range(2):
            if self.ai:
                green_dots = self.detect_with_yolo("green_front")
            else:
                green_dots = self._getBallishThing(self._green_lower_limit, self._green_upper_limit, self._dotSizeLower,
                                                   self._dotSizeUpper)

            # print("Looking for green dots. Current number of green dots: " + str(len(green_dots)))
            if len(green_dots) == 1:
                break
        if len(green_dots) > 1:
            raise NoRobotException("More than one green dot")
        elif len(green_dots) == 0:
            raise NoRobotException("No green dots")
        green_dot = green_dots[0]

        blue_dot = blue_dots[0]
        center = CircleObject(radius=1,
                              position=Position(x=int((green_dot.position.x + blue_dot.position.x) / 2),
                                                y=int((green_dot.position.y + blue_dot.position.y) / 2)))
        angle_xy = calculate_positive_angle(green_dot, blue_dot)
        center = self._correct_robot_location_perspective(center)
        return center, angle_xy

    def _get_white_balls(self) -> List[CircleObject]:
        if self.ai:
            return self.detect_with_yolo("white_ball")
        else:
            return self._getBallishThing(self._whiteLower, self._whiteUpper, self._whiteSizeLower, self._whiteSizeUpper)


    def _get_orange_ball(self, decrease_tolerance=False) -> List[CircleObject]:
        if self.ai:
            return self.detect_with_yolo("orange_ball")
        else:
            return self._getBallishThing(self._orange_lower_limit, self._orange_upper_limit,
                                                 self._whiteSizeLower,
                                                 self._whiteSizeUpper, decrease_tolerance)

    def _get_egg(self) -> List[CircleObject]:
        if self.ai:
            return self.detect_with_yolo("egg")
        else:
            return self._getBallishThing(self._whiteLower, self._whiteUpper, self._eggSizeLower, self._eggSizeUpper)

    def _get_robot_square(self) -> SquareObject or None:
        try:
            circle, angle = self._get_robot_center()
            square = SquareObject.create_square(circle.position,
                                                self._robot_y,
                                                self._robot_x,
                                                angle
                                                )
        except Exception as e:
            print("Failed to locate robot: " + str(e))
            return None
        return square

    def get_any_thing(self, min_count=0, max_count=100000, tries=50, thing_to_get=""):

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
        elif thing_to_get == "all_balls":
            func = self._get_all_balls
        else:
            raise Exception("Invalid argument")

        if thing_to_get != "robot":
            for _ in range(tries):
                list_of_thing = func()
                if min_count <= len(list_of_thing) <= max_count:
                    return list_of_thing
            # print(thing_to_get + " not found within parameters")
        else:
            robot = func()
            while not robot and tries > 0:
                robot = func()
                tries = tries - 1
            return robot

    def _get_all_balls(self):
        self.commonSetup()
        ret, frame = self._vs.read()
        results = self.model.predict(frame, conf=0.3, iou=0.3)
        orange_balls = []
        white_balls = []
        blue_labels = []
        green_labels = []
        num_white_balls = 0
        num_orange_balls = 0
        for result in results:
            for box in result.boxes:
                cls = result.names[box.cls[0].item()]
                if cls == "orange_ball":
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    radius = (x2 - x1 + y2 - y1) // 4  # Approximate radius
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    orange_balls.append(CircleObject(radius=radius, position=Position(x=center_x, y=center_y)))
                    num_orange_balls += 1
                elif cls == "white_ball":
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    radius = (x2 - x1 + y2 - y1) // 4  # Approximate radius
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    white_balls.append(CircleObject(radius=radius, position=Position(x=center_x, y=center_y)))
                    num_white_balls += 1
                elif cls == "green_front":
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    radius = (x2 - x1 + y2 - y1) // 4  # Approximate radius
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    green_labels.append(CircleObject(radius=radius, position=Position(x=center_x, y=center_y)))
                elif cls == "purple_back":
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    radius = (x2 - x1 + y2 - y1) // 4  # Approximate radius
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    blue_labels.append(CircleObject(radius=radius, position=Position(x=center_x, y=center_y)))

        print("num_white_balls in image:", num_orange_balls)
        print("num_orange_balls in image:", num_orange_balls)

        robot_square: SquareObject = self._get_robot_square_ai(green_labels=green_labels, blue_labels=blue_labels)

        if len(orange_balls) != 0:
            return orange_balls, robot_square
        else:
            return white_balls, robot_square

    def _get_robot_square_ai(self, green_labels: [CircleObject], blue_labels: [CircleObject]) -> SquareObject or None:
        try:
            circle, angle = self._get_robot_center_ai(green_labels=green_labels, blue_labels=blue_labels)
            square = SquareObject.create_square(circle.position,
                                                self._robot_y,
                                                self._robot_x,
                                                angle
                                                )
            self.last_robot_square = square
        except Exception as e:
            # print("Failed to locate robot: " + str(e))
            if self.last_robot_square == None:
                return SquareObject.create_square(Position(x=0, y=0), 0, 0, 0)
            return self.last_robot_square
        return square

    def _get_robot_center_ai(self, green_labels: [CircleObject], blue_labels: [CircleObject]) -> Tuple[CircleObject, float]:
        green_dots = green_labels
        blue_dots = blue_labels
        # TODO: These loops should probably just be removed. I dont want to retry extensively here,
        # Because retrying 30 times (one second) could cause significant desync between the two dots,
        # leading to a misrepresented location

        if len(blue_dots) > 1:
            raise NoRobotException("More than one blue dot")
        elif len(blue_dots) == 0:
            raise NoRobotException("No blue dots")

        if len(green_dots) > 1:
            raise NoRobotException("More than one green dot")
        elif len(green_dots) == 0:
            raise NoRobotException("No green dots")
        green_dot = green_dots[0]

        blue_dot = blue_dots[0]
        center = CircleObject(radius=1,
                              position=Position(x=int((green_dot.position.x + blue_dot.position.x) / 2),
                                                y=int((green_dot.position.y + blue_dot.position.y) / 2)))
        self.orientation = calculate_positive_angle(green_dot, blue_dot)
        angle_xy = calculate_positive_angle(green_dot, blue_dot) # (math.pi * 2) -
        center = self._correct_robot_location_perspective(center)
        return center, angle_xy

    def calculate_angle(self, dot1: CircleObject, dot2: CircleObject) -> float:
        delta_x = dot2.position.x - dot1.position.x
        delta_y = dot2.position.y - dot1.position.y
        angle = math.atan2(delta_y, delta_x)

        # Ensure the angle is within the range [-2*pi, 2*pi]
        if angle < -2 * math.pi:
            angle += 2 * math.pi
        elif angle > 2 * math.pi:
            angle -= 2 * math.pi

        return angle

    def detect_with_yolo(self, thing_type: str) -> List[CircleObject]:
        self.commonSetup()
        ret, frame = self._vs.read()
        results = self.model.predict(frame, conf=0.3, iou=0.3)
        detected_objects = []
        for result in results:
            for box in result.boxes:
                cls = result.names[box.cls[0].item()]
                if cls == thing_type:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    radius = (x2 - x1 + y2 - y1) // 4  # Approximate radius
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    detected_objects.append(CircleObject(radius=radius, position=Position(x=center_x, y=center_y)))
        return detected_objects

if __name__ == '__main__':
    pass
