from pydantic import BaseModel, confloat
from typing import Optional, List, Tuple
import math
from numpy import np



class Position(BaseModel):
    x: float
    y: float

class SquareObject(BaseModel):
    position: Position
    width: int
    height: int
    verticies: List[Tuple[float, float]]

    def __init__(self, position: Position, width: int, height: int, radians: float):
        vertices = self.update_square(position, radians)
        super().__init__(position=position, width=width, height=height, vertices=vertices)

    def update_square(self, position: Position, radians: float):
        half_width = self.width / 2
        half_height = self.height / 2
        vertices = [
            (position.x - half_width, position.y - half_height),
            (position.x - half_width, position.y + half_height),
            (position.x + half_width, position.y + half_height),
            (position.x + half_width, position.y - half_height)
        ]
        self.position=position
        if radians:
            vertices = rotate_square(vertices=vertices)
        return vertices
    

class CircleObject(BaseModel):
    radius: int
    position: Position

    def contains_point(self, point: Position) -> bool:
        return math.hypot(self.position.x - point.x, self.position.y - point.y) <= self.radius
    


class Player(BaseModel):
    player: SquareObject
    suction: SquareObject
    radian: float
    collected_balls: List[CircleObject]

    def __init__(self, position: Position, width: int, height: int, radians: float, 
                suction_width: int, suction_height: int, suction_offset_x: int=0, suction_offset_y: int=0):

        if not 0 < radians < 2 * math.pi:
            raise Exception("Number must be within range 0 to 2 * pi")

        super.__init__(self(position=position, 
                   width=width, 
                   height=height, 
                   direction=radians, 
                   suction_width=suction_width,
                   suction_height=suction_height,
                   suction_offset_x=suction_offset_x,
                   suction_offset_y=suction_offset_y,
                   collected_balls=[]))

    def check_for_wall(self, verticies: List[Tuple[float, float]]):
        for point in verticies:
            self.player.contains_point(Position(x=point[0], y=point[0]))

    def move(self, dx: int, dy: int, ddirection: float):
        self.player.position.x += dx
        self.player.position.y += dy
        self.radian += ddirection

    def contains_point(self, point: Position) -> bool:
        return self.position.x - self.width / 2 <= point.x <= self.position.x + self.width / 2 \
            and self.position.y - self.height / 2 <= point.y <= self.position.y + self.height / 2

    def collect_balls(self, balls: List[CircleObject]) -> List[CircleObject]:
        for ball in balls:
            if self.contains_point(ball.position):
                self.collected_balls.append(ball)
        return self.collected_balls
    


def rotate_square(vertices: List[Tuple[float, float]], center: Position, radians: float) -> List[Tuple[float, float]]:
    cx = center.x
    cy = center.y
    
    rotation_matrix = np.array([
        [np.cos(radians), -np.sin(radians)],
        [np.sin(radians),  np.cos(radians)]
    ])
    
    translated_vertices = vertices - np.array([cx, cy])
    
    rotated_vertices = np.dot(translated_vertices, rotation_matrix)
    
    rotated_vertices += np.array([cx, cy])

    return rotated_vertices