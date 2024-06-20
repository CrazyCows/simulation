from pydantic import BaseModel, confloat
from typing import Optional, List, Tuple
import math
import numpy as np
from enum import Enum


class WallPosition(Enum):
    UNDEFINED = -1
    LEFT = 0
    RIGHT = 1
    TOP = 2
    BOT = 3


class Position(BaseModel):
    x: float
    y: float


class LineObject(BaseModel):
    start_pos: Position
    end_pos: Position


class SquareObject(BaseModel):
    position: Position
    width: int
    height: int
    radians: float
    vertices: List[Tuple[float, float]]
    offset_x: int
    offset_y: int
    wallPosition: WallPosition
    danger_zone: bool

    def update_square(self, position: Position, radians: float = 0):
        # Apply offset to the position
        position_with_offset = Position(x=position.x + self.offset_x, y=position.y + self.offset_y)
        self.position = position_with_offset
        half_width = self.width / 2
        half_height = self.height / 2
        self.radians = radians
        self.vertices = [
            (position_with_offset.x - half_width, position_with_offset.y - half_height),
            (position_with_offset.x - half_width, position_with_offset.y + half_height),
            (position_with_offset.x + half_width, position_with_offset.y + half_height),
            (position_with_offset.x + half_width, position_with_offset.y - half_height)
        ]

        self.vertices = rotate_square(vertices=self.vertices, center=position, radians=radians)

    @classmethod
    def create_square(cls, position: Position, width: int, height: int, radians: float, offset_x=0, offset_y=0,
                      danger_zone=False, wallPosition=-1):
        position_with_offset = Position(x=position.x + offset_x, y=position.y + offset_y)

        half_width = width / 2
        half_height = height / 2
        vertices = [
            (position_with_offset.x - half_width, position_with_offset.y - half_height),
            (position_with_offset.x - half_width, position_with_offset.y + half_height),
            (position_with_offset.x + half_width, position_with_offset.y + half_height),
            (position_with_offset.x + half_width, position_with_offset.y - half_height)
        ]
        vertices = rotate_square(vertices=vertices, center=position, radians=radians)

        return cls(position=position,
                   width=width,
                   height=height,
                   radians=radians,
                   offset_x=offset_x,
                   offset_y=offset_y,
                   vertices=vertices,
                   danger_zone=danger_zone,
                   wallPosition=WallPosition(wallPosition))

    def center_of_longest_side(self):
        def distance(p1, p2):
            return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

        longest_side_center = None
        max_length = 0

        for i in range(4):
            p1 = self.vertices[i]
            p2 = self.vertices[(i + 1) % 4]
            length = distance(p1, p2)
            if length > max_length:
                max_length = length
                longest_side_center = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)

        return longest_side_center


class CircleObject(BaseModel):
    radius: int
    position: Position

    def contains_point(self, point: Position) -> bool:
        return math.hypot(self.position.x - point.x, self.position.y - point.y) <= self.radius




def rotate_square(vertices: List[Tuple[float, float]], center: Position, radians: float) -> List[Tuple[float, float]]:
    """
            Rotates verticies around a center using matrix multiplication with np array to minimize resource usage.
        """
    cx = center.x
    cy = center.y

    rotation_matrix = np.array([
        [np.cos(radians), -np.sin(radians)],
        [np.sin(radians), np.cos(radians)]
    ])

    translated_vertices = vertices - np.array([cx, cy])

    rotated_vertices = np.dot(translated_vertices, rotation_matrix)

    rotated_vertices += np.array([cx, cy])

    return rotated_vertices
