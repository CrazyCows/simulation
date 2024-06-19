import math
from typing import Tuple

import cv2
from dto.shapes import SquareObject, Position, CircleObject
import image_recognizition.object_detection as od


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
        self.cap = cv2.VideoCapture(0)
        self.frame_name = "Placeholder_name"

    def _click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append((x, y))
            ret, self.frame = self.cap.read()
            cv2.circle(self.frame, (x, y), 5, (0, 255, 0), -1)
            cv2.imshow(self.frame_name, self.frame)
            if len(self.points) >= self.max_points:
                cv2.destroyAllWindows()

    def _pick_points(self, window_name) -> SquareObject:
        num_points = 4
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
        pos = Position(x=self.points[0][1], y=self.points[0][1])
        p1 = CircleObject(radius=1, position=pos)
        p2 = CircleObject(radius=1, position=Position(x=self.points[1][1], y=self.points[1][1]))
        angle = od.calculate_positive_angle(p1, p2)
        width = int(euclidean_distance(self.points[0], self.points[1]))
        height = int(euclidean_distance(self.points[1], self.points[2]))

        return SquareObject(position=pos,
                            width=width,
                            height=height,
                            radians=angle,
                            vertices=self.points,
                            offset_x=0,
                            offset_y=0)

    def _pick_north_wall(self):
        print("Click 4 points for the North Wall")
        return self._pick_points("North")

    def _pick_east_wall(self):
        print("Click 4 points for the East Wall")
        return self._pick_points("East")

    def _pick_south_wall(self):
        print("Click 4 points for the South Wall")
        return self._pick_points("South")

    def _pick_west_wall(self):
        print("Click 4 points for the West Wall")
        return self._pick_points("West")

    def _click_cross_one(self):
        print("Click 4 points for Cross One")
        return self._pick_points("Cross part one")

    def _click_cross_two(self):
        print("Click 4 points for Cross Two")
        return self._pick_points("Cross part two")

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    wp = WallPicker()
    try:
        north_wall_points = wp._pick_north_wall()
        print("North Wall Points:", north_wall_points)

        east_wall_points = wp._pick_east_wall()
        print("East Wall Points:", east_wall_points)

        south_wall_points = wp._pick_south_wall()
        print("South Wall Points:", south_wall_points)

        west_wall_points = wp._pick_west_wall()
        print("West Wall Points:", west_wall_points)

        cross_one_points = wp._click_cross_one()
        print("Cross One Points:", cross_one_points)

        cross_two_points = wp._click_cross_two()
        print("Cross Two Points:", cross_two_points)
    finally:
        wp.release()
