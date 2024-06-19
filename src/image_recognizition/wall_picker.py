import cv2
from dto.shapes import SquareObject, Position
from image_recognizition import object_detection as od

class WallPicker:
    def __init__(self):
        self.points = []
        self.frame = None
        self.cap = cv2.VideoCapture(0)

    def _click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append((x, y))
            cv2.circle(self.frame, (x, y), 5, (0, 255, 0), -1)
            cv2.imshow('Webcam Feed', self.frame)
            if len(self.points) >= self.max_points:
                cv2.destroyAllWindows()

    def _pick_points(self, num_points, window_name) -> SquareObject:
        self.points = []
        self.max_points = num_points
        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self._click_event)

        while True:
            ret, self.frame = self.cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break
            cv2.imshow('Webcam Feed', self.frame)
            if cv2.waitKey(1) & 0xFF == ord('q') or len(self.points) >= self.max_points:
                break
        squares = []
        angle = od.calculate_positive_angle(self.points[1], self.points[0])
        for index, point in self.points:

            squares.append(SquareObject(x=point[0], y=point[1], angle=angle))
        return self.points

    def _pick_north_wall(self):
        print("Click 4 points for the North Wall")
        return self._pick_points(4, "North")

    def _pick_east_wall(self):
        print("Click 4 points for the East Wall")
        return self._pick_points(4, "East")

    def _pick_south_wall(self):
        print("Click 4 points for the South Wall")
        return self._pick_points(4, "South")

    def _pick_west_wall(self):
        print("Click 4 points for the West Wall")
        return self._pick_points(4, "West")

    def _click_cross_one(self):
        print("Click 4 points for Cross One")
        return self._pick_points(4, "Cross part one")

    def _click_cross_two(self):
        print("Click 4 points for Cross Two")
        return self._pick_points(4, "Cross part two")

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