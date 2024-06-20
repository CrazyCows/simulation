from typing import List, Tuple
import math
import numpy as np
from dto.shapes import SquareObject, CircleObject, LineObject, Position




def square_touching(square1: SquareObject, square2: SquareObject) -> bool:
    """
        Checks if two squares is touching
    """
    def project(vertices, axis):
        dots = [np.dot(vertex, axis) for vertex in vertices]
        return [min(dots), max(dots)]

    def overlap(proj1, proj2):
        return proj1[1] >= proj2[0] and proj2[1] >= proj1[0]

    def separating_axis_theorem(vertices1, vertices2):
        axes = []
        for i in range(len(vertices1)):
            p1 = vertices1[i]
            p2 = vertices1[(i + 1) % len(vertices1)]
            edge = np.subtract(p2, p1)
            normal = [-edge[1], edge[0]]
            axes.append(normal)
        for i in range(len(vertices2)):
            p1 = vertices2[i]
            p2 = vertices2[(i + 1) % len(vertices2)]
            edge = np.subtract(p2, p1)
            normal = [-edge[1], edge[0]]
            axes.append(normal)
        
        for axis in axes:
            proj1 = project(vertices1, axis)
            proj2 = project(vertices2, axis)
            if not overlap(proj1, proj2):
                return False
        return True
    
    return separating_axis_theorem(square1.vertices, square2.vertices)
        

def circle_touch(circle1: CircleObject, circle2: CircleObject) -> bool:
    """
        Checks if two circles is touching
    """
    speed = math.hypot(circle1.position.x - circle2.position.x, circle1.position.y - circle2.position.y)
    radius_sum = circle1.radius + circle2.radius
    tolerance = 0.0001
    return abs(speed - radius_sum) <= tolerance


def circle_square_touch(circle: CircleObject, square: SquareObject) -> bool:
    """
        Checks if a square and a circle is touching
    """
    def circle_line_segment_touch(circle: CircleObject, p1: Position, p2: Position) -> bool:
        """
            Checks if a segment of a square is touching
        """

        cx, cy, r = circle.position.x, circle.position.y, circle.radius
        x1, y1 = p1.x, p1.y
        x2, y2 = p2.x, p2.y

        dx, dy = x2 - x1, y2 - y1
        fx, fy = x1 - cx, y1 - cy

        a = dx * dx + dy * dy
        b = 2 * (fx * dx + fy * dy)
        c = (fx * fx + fy * fy) - r * r

        discriminant = b * b - 4 * a * c
        if discriminant >= 0:
            discriminant = math.sqrt(discriminant)
            t1 = (-b - discriminant) / (2 * a)
            t2 = (-b + discriminant) / (2 * a)

            if 0 <= t1 <= 1 or 0 <= t2 <= 1:
                return True

        return False

    for i in range(len(square.vertices)):
        p1 = Position(x=square.vertices[i][0], y=square.vertices[i][1])
        p2 = Position(x=square.vertices[(i + 1) % len(square.vertices)][0], y=square.vertices[(i + 1) % len(square.vertices)][1])
        if circle_line_segment_touch(circle, p1, p2):
            return True

    return False

def calculate_coordinates_for_line(direction, start_x, start_y, length=1200):
    """
        Calculates a straight line from coordiantes with a chosen length and angle.
    """

    
    end_x = start_x + length * math.sin(direction)
    end_y = start_y + length * math.cos(direction)
    
    return LineObject(start_pos=Position(x=start_x, y=start_y), end_pos=Position(x=end_x, y=end_y))
