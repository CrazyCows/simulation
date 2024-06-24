import math
from typing import Tuple
import numpy as np
import cv2
from dto.shapes import SquareObject, Position, CircleObject
from copy import deepcopy
from dto.obstacles import Cross, Wall, WallPlacement
from dto.robot import Checkpoint


def calculate_positive_angle(circle1: CircleObject, circle2: CircleObject) -> float:
    # IDK if its necessary to convert to np arrays
    pos1 = np.array([circle1.position.x, circle1.position.y])
    pos2 = np.array([circle2.position.x, circle2.position.y])
    delta = pos2 - pos1
    # These are switched per "(Note the role reversal: the “y-coordinate” is the first function parameter, the “x-coordinate” is the second."
    # - numpy docs
    angle = np.arctan2(delta[1], delta[0])
    angle = -angle
    if angle < 0:
        angle += 2 * np.pi

    #Error checking for float inaccuracies
    elif angle > 2 * np.pi:
        angle = angle - 2 * np.pi
    return angle

def calculate_centroid(points):
    if not points:
        return None

    x_coords = [p[0] for p in points]
    y_coords = [p[1] for p in points]

    centroid_x = sum(x_coords) / len(points)
    centroid_y = sum(y_coords) / len(points)

    return (centroid_x, centroid_y)


def euclidean_distance(point1: Tuple[int, int], point2: Tuple[int, int]) -> float:
    # Unpack the points
    x1, y1 = point1
    x2, y2 = point2

    # Calculate the difference in x and y coordinates
    dx = x2 - x1
    dy = y2 - y1

    # Calculate the Euclidean distance
    distance = math.sqrt(dx * dx + dy * dy)
    return distance


class WallPicker:
    max_points = 0
    points = []


    def __init__(self):
        self.points = []
        self.frame = None
        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.frame_name = "Placeholder_name"

    def _click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append((x, y))
            ret, self.frame = self.cap.read()
            cv2.circle(self.frame, (x, y), 5, (0, 255, 0), -1)
            cv2.imshow(self.frame_name, self.frame)
            if len(self.points) >= self.max_points:
                cv2.destroyAllWindows()

    def _pick_points(self, window_name, num_points) -> SquareObject:
        self.points = []
        self.max_points = num_points
        self.frame_name = window_name
        cv2.namedWindow(self.frame_name)
        cv2.setMouseCallback(self.frame_name, self._click_event)


        while True:

            ret, self.frame = self.cap.read()
            cv2.imshow(self.frame_name, self.frame)

            if not ret:
                print("Error: Could not read frame.")
                break

            if cv2.waitKey(1) & 0xFF == ord('q') or len(self.points) >= num_points:
                break
        print(self.points)

        p1 = CircleObject(radius=1, position=Position(x=self.points[0][0], y=self.points[0][1]))
        p2 = CircleObject(radius=1, position=Position(x=self.points[1][0], y=self.points[1][1]))
        angle = calculate_positive_angle(p1, p2)
        width = int(euclidean_distance(self.points[0], self.points[1]))
        height = int(euclidean_distance(self.points[1], self.points[2]))
        x, y = calculate_centroid(self.points)
        print(x,y)



        return SquareObject.create_square(
                            position=Position(x=x, y=y),
                            width=width,
                            height=height,
                            radians=angle,
                            offset_x=0,
                            offset_y=0)

    def pick_north_wall(self):
        print("Click 4 points for the North Wall")
        square = self._pick_points("North", 4)
        wall = Wall.create(square, WallPlacement.TOP)
        return wall

    def pick_east_wall(self):
        print("Click 4 points for the East Wall")
        square = self._pick_points("East", 4)
        wall = Wall.create(square, WallPlacement.LEFT)
        return wall

    def pick_south_wall(self):
        print("Click 4 points for the South Wall")
        square = self._pick_points("South", 4)
        wall = Wall.create(square, WallPlacement.BOT)
        return wall
    def pick_west_wall(self):
        print("Click 4 points for the West Wall")
        square = self._pick_points("West", 4)
        wall = Wall.create(square, WallPlacement.RIGHT)
        return wall

    def _click_cross_one(self):
        print("Click 4 points for Cross One")
        return self._pick_points("One rectangle from cross", 4)

    def pick_hole(self):
        print("Click the two of the hole")
        points = self._pick_points("Hole", 2)
        x, y = calculate_centroid(points)
        return CircleObject(radius=1, position=Position(x=x, y=y))

    def pick_cross(self):
        cross_part_one: SquareObject = self._click_cross_one()
        cross_part_two: SquareObject = deepcopy(cross_part_one)
        cross_part_two.update_square(position=cross_part_two.position, radians=cross_part_two.radians+np.pi/2)
        if cross_part_two.radians > 2*np.pi:
            cross_part_two.radians = cross_part_two.radians - 2*np.pi
        #cross = Cross.create_cross_with_safe_zones(cross_part_one, cross_part_two, walls)
        return [cross_part_one, cross_part_two]

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    pass
