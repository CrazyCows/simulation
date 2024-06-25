from dto.shapes import SquareObject, CircleObject, Position
from pydantic import BaseModel
from typing import List, Optional
from enum import Enum
import math

class WallPlacement(Enum):
    LEFT = "left"
    RIGHT = "right"
    TOP = "up"
    BOT = "down"
    CROSS = "cross"

class Wall(SquareObject):
    placement: WallPlacement
    danger_zone: SquareObject

    @classmethod
    def create(cls, square_object: SquareObject, placement: WallPlacement, danger_zone_size: int = 40):
        return cls(
            position=Position(x=square_object.position.x, y=square_object.position.y),
            width=square_object.width,
            height=square_object.height,
            radians=square_object.radians,
            vertices=square_object.vertices,
            offset_x=square_object.offset_x,
            offset_y=square_object.offset_y,
            placement=placement,
            danger_zone=SquareObject.create_square(
                position=Position(x=square_object.position.x, y=square_object.position.y),
                width=square_object.width + danger_zone_size,
                height=square_object.height + danger_zone_size,
                radians=square_object.radians,
                offset_x=square_object.offset_x,
                offset_y=square_object.offset_y
            )
        )


class Cross(BaseModel):
    square_1: Wall
    square_2: Wall
    safe_zones: List[Position]

    def __iter__(cls):
        yield cls.square_1
        yield cls.square_2

    @classmethod
    def create_cross_with_safe_zones(cls, square_1: SquareObject, square_2: SquareObject, walls: List[SquareObject], safe_distance: float =200):
        center_1 = square_1.position
        center_2 = square_2.position

        # Find midpoint between the two squares to use as center of the cross
        center_x = (center_1.x + center_2.x) / 2
        center_y = (center_1.y + center_2.y) / 2
        cross_center = Position(x=center_x, y=center_y)

        # Define the directions of the safe zones at 45-degree angles
        directions = [
            (math.cos(math.radians(0)), math.sin(math.radians(0))),
            (math.cos(math.radians(45)), math.sin(math.radians(45))),
            (math.cos(math.radians(90)), math.sin(math.radians(90))),
            (math.cos(math.radians(135)), math.sin(math.radians(135))),
            (math.cos(math.radians(180)), math.sin(math.radians(180))),
            (math.cos(math.radians(225)), math.sin(math.radians(225))),
            (math.cos(math.radians(270)), math.sin(math.radians(270))),
            (math.cos(math.radians(315)), math.sin(math.radians(315)))
        ]

        safe_zones = []
        
        for i, direction in enumerate(directions):
            direction_x, direction_y = direction

            # Calculate the distance to the nearest wall in the direction of the safe zone
            max_distance = float('inf')
            for wall in walls:
                for vertex in wall.vertices:
                    wall_vertex_pos = Position(x=vertex[0], y=vertex[1])
                    distance_x = (wall_vertex_pos.x - cross_center.x) / direction_x if direction_x != 0 else float('inf')
                    distance_y = (wall_vertex_pos.y - cross_center.y) / direction_y if direction_y != 0 else float('inf')
                    distance = min(abs(distance_x), abs(distance_y)) - safe_distance
                    max_distance = min(max_distance, distance)
            
            # Left and right point
            if i == 0 or i == 4:
                safe_zone_x = cross_center.x + direction_x * max_distance * 0.65
                safe_zone_y = cross_center.y + direction_y * max_distance * 0.65
                safe_zones.append(Position(x=safe_zone_x, y=safe_zone_y))
            
            # Up and down point
            if i == 2 or i == 6:
                safe_zone_x = cross_center.x + direction_x * max_distance * 0.7
                safe_zone_y = cross_center.y + direction_y * max_distance * 0.7
                safe_zones.append(Position(x=safe_zone_x, y=safe_zone_y))
            
            if i == 3 or i == 7:
                safe_zone_x = cross_center.x + direction_x * max_distance * 0.7
                safe_zone_y = cross_center.y + direction_y * max_distance * 0.7
                safe_zones.append(Position(x=safe_zone_x, y=safe_zone_y))
            
            if i== 1 or i == 5:
                safe_zone_x = cross_center.x + direction_x * max_distance * 0.7
                safe_zone_y = cross_center.y + direction_y * max_distance * 0.7
                safe_zones.append(Position(x=safe_zone_x, y=safe_zone_y))
            
        
            
        return cls(square_1=Wall.create(square_object=square_1, placement=WallPlacement.CROSS, danger_zone_size=40), square_2=Wall.create(square_object=square_2, placement=WallPlacement.CROSS, danger_zone_size=40), safe_zones=safe_zones)


if __name__ == '__main__':
    # Example usage
    square1 = SquareObject.create_square(position=Position(x=0, y=0), width=10, height=2, radians=0)
    square2 = SquareObject.create_square(position=Position(x=0, y=10), width=2, height=10, radians=0)
    walls = [
        SquareObject.create_square(position=Position(x=20, y=0), width=5, height=5, radians=0),
        SquareObject.create_square(position=Position(x=-20, y=0), width=5, height=5, radians=0),
        SquareObject.create_square(position=Position(x=0, y=20), width=5, height=5, radians=0),
        SquareObject.create_square(position=Position(x=0, y=-20), width=5, height=5, radians=0)
    ]
    cross = Cross.create_cross_with_safe_zones(square_1=square1, square_2=square2, walls=walls, safe_distance=2)
    print(cross)