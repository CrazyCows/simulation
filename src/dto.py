from pydantic import BaseModel
from typing import Optional, List
import math

class Position(BaseModel):
    x: float
    y: float

class SquareObject(BaseModel):
    position: Position
    width: int
    height: int
    degrees: float

    def contains_point(self, point: Position) -> bool:
        return self.position.x - self.width / 2 <= point.x <= self.position.x + self.width / 2 \
            and self.position.y - self.height / 2 <= point.y <= self.position.y + self.height / 2

class CircleObject(BaseModel):
    radius: int
    position: Position

    def contains_point(self, point: Position) -> bool:
        return math.hypot(self.position.x - point.x, self.position.y - point.y) <= self.radius
    
class Wall(BaseModel):
    position: Position
    width: int
    height: int 

class Player(BaseModel):
    position: Position
    width: int
    height: int
    direction: float
    suction_width: int
    suction_height: int
    suction_offset_x: int
    suction_offset_y: int
    collected_balls: List[CircleObject]

    def move(self, dx: int, dy: int, ddirection: float):
        self.position.x += dx
        self.position.y += dy
        self.direction += ddirection

    def contains_point(self, point: Position) -> bool:
        return self.position.x - self.width / 2 <= point.x <= self.position.x + self.width / 2 \
            and self.position.y - self.height / 2 <= point.y <= self.position.y + self.height / 2

    def collect_balls(self, balls: List[CircleObject]) -> List[CircleObject]:
        for ball in balls:
            if self.contains_point(ball.position):
                self.collected_balls.append(ball)
        return self.collected_balls
    
    def contains_wall(self, walls: List[Wall]):
        for wall in walls:
            ""
    
    @classmethod
    def create_player(cls, position: Position, width: int, height: int, degrees: float, 
                      suction_width: int, suction_height: int, suction_offset_x: int=0, suction_offset_y: int=0):
        return cls(position=position, 
                   width=width, 
                   height=height, 
                   direction=degrees, 
                   suction_width=suction_width,
                   suction_height=suction_height,
                   suction_offset_x=suction_offset_x,
                   suction_offset_y=suction_offset_y,
                   collected_balls=[])
    
