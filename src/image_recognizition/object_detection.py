import cv2
import imutils
import numpy as np
from cv2.gapi.wip.draw import Circle
from typing import List, Tuple
from dto.shapes import CircleObject, Position, SquareObject

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

    def __init__(self, walls: List[SquareObject]):
        for wall in walls:
            print("")
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

    _purple_lower_limit = np.array([140, 150, 70])
    _purple_upper_limit = np.array([160, 255, 255])

    _new_green_lower_limit = np.aray([40, 100, 100])
    _new_green_upper_limit = np.array([80, 255, 255])

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
    _vs = cv2.VideoCapture(0)
    _vs.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    _vs.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # Variables to be updated every darn iteration
    _blurred = None
    _hsv = None

    def get_flipped_frame(self):
        ret, frame = self._vs.read()
        cv2.flip(frame, 1, frame)
        return frame

    def commonSetup(self):
        ret, frame = self._vs.read()

        if frame is None:
            raise Exception("Camera error")
        # print(type(frame))
        # print("Dimensions:" + str(frame.shape))
        # frame = frame[int(self._min_y):int(self._max_y), int(self._min_x):int(self._max_x)]
        # print(type(frame))
        # print("Dimensions:" + str(frame.shape))

        if self._camera_x is None:
            self._camera_x = int(self._vs.get(cv2.CAP_PROP_FRAME_WIDTH) / 2)
            self._camera_y = int(self._vs.get(cv2.CAP_PROP_FRAME_HEIGHT) / 2)
        frame = increase_vibrance(frame, 1.5)
        self._blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        self._hsv = cv2.cvtColor(self._blurred, cv2.COLOR_BGR2HSV)

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
        print(approximations)
        for approx in approximations:
            for thing in approx:
                print("x: " + str(thing[0][0]))
                print("y: " + str(thing[0][1]))

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

    def _get_robot_center_qr_code(self, backward: bool):
        qr_codes_final = []
        if not self._vs.isOpened():
            print("Error: Could not open video stream.")
        else:
            # Capture frame-by-frame
            ret, frame = self._vs.read()
            

            # Detect QR Codes in the image
            qr_codes = pyzbar.decode(frame)

            # Process detected QR Codes
            for qr in qr_codes:
                # Extract data and rectangle corners of the QR code
                qr_data = qr.data.decode('utf-8')
                if qr_data.lower() == "FREMAD".lower() and backward:
                    continue
                elif qr_data.lower() == "TILBAGE".lower() and not backward:
                    continue

                (x, y, w, h) = qr.rect
                center_x, center_y = x + w // 2, y + h // 2
                
                # Draw rectangle around the QR code
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                # Draw circle at the center
                cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

                # Display the QR code data
                text = f"Data: {qr_data} | Center: ({center_x}, {center_y})"
                cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                qr_codes_final.append(CircleObject(radius=30, position=Position(x=center_x, y=center_y)))
            # Display the resulting frame
            cv2.imshow('DroidCam Video', frame)

            return qr_codes_final
                


    def _get_robot_center(self) -> Tuple[CircleObject, float]:
        

        purple_dots = []
        new_green_dots = []

        purple_dots = self._getBallishThing(self._purple_lower_limit, self._purple_upper_limit, self._dotSizeLower,
                                             self._dotSizeUpper)
        if len(purple_dots) != 1:
            raise NoRobotException("Number of purple dots: " + str(len(purple_dots)))
        new_green_dots = self._getBallishThing(self._new_green_lower_limit, self._new_green_upper_limit, self._dotSizeLower,
                                             self._dotSizeUpper)
        if len(new_green_dot) != 1:
            raise NoRobotException("Number of new green dots: " + str(len(new_green_dots)))
        
        purple_dot = purple_dots[0]
        new_green_dot = purple_dots[0]
        center = CircleObject(radius=1,
                              position=Position(x=int((purple_dot.position.x + new_green_dot.position.x) / 2),
                                                y=int((new_green_dot.position.y + purple_dot.position.y) / 2)))
        angle_xy = calculate_positive_angle(purple_dot, new_green_dot)
        center = self._correct_robot_location_perspective(center)
        return center, angle_xy
        # TODO: These loops should probably just be removed. I dont want to retry extensively here,
        # Because retrying 30 times (one second) could cause significant desync between the two dots,
        # leading to a misrepresented location
        """
        green_dots = []
        blue_dots = []
        for i in range(2):
            blue_dots = self._getBallishThing(self._blue_lower_limit, self._blue_upper_limit, self._dotSizeLower,
                                             self._dotSizeUpper)
            #blue_dots = self._get_robot_center_qr_code(True)
            # print("Looking for blue dot. Current number of blue dots: " + str(len(blue_dots)))
            if len(blue_dots) == 1:
                break
        if len(blue_dots) > 1:
            #raise NoRobotException("More than one blue dot")
            pass #TODO
        elif len(blue_dots) == 0:
            raise NoRobotException("No blue dots")
        for i in range(2):
            green_dots = self._getBallishThing(self._green_lower_limit, self._green_upper_limit, self._dotSizeLower,
                                              self._dotSizeUpper)
            #green_dots = self._get_robot_center_qr_code(False)
            # print("Looking for green dots. Current number of green dots: " + str(len(green_dots)))
            if len(green_dots) == 1:
                break
        if len(green_dots) > 1:
            #raise NoRobotException("More than one green dot")
            pass #TODO revert
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
        """
        

    def _get_white_balls(self) -> List[CircleObject]:
        return self._getBallishThing(self._whiteLower, self._whiteUpper, self._whiteSizeLower, self._whiteSizeUpper)

    def _get_orange_ball(self, decrease_tolerance=False) -> List[CircleObject]:
        orange_balls = self._getBallishThing(self._orange_lower_limit, self._orange_upper_limit, self._whiteSizeLower,
                                             self._whiteSizeUpper, decrease_tolerance)
        return orange_balls

    def _get_egg(self) -> List[CircleObject]:
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
            while not robot and tries > 0:
                robot = func()
                tries = tries - 1
            return robot


if __name__ == '__main__':
    pass
