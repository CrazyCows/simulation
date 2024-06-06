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
    offset_x: Optional[int] = None
    offset_y: Optional[int] = None

    def contains_point(self, point: Position) -> bool:
        return self.position.x - self.width / 2 <= point.x <= self.position.x + self.width / 2 \
            and self.position.y - self.height / 2 <= point.y <= self.position.y + self.height / 2

class CircleObject(BaseModel):
    radius: int
    position: Position

    def contains_point(self, point: Position) -> bool:
        return math.hypot(self.position.x - point.x, self.position.y - point.y) <= self.radius
    
class Player(BaseModel):
    position: Position
    player_area: SquareObject
    suction_area: SquareObject
    collected_balls: List[CircleObject]

    def move(self, dx: int, dy: int):
        self.position.x += dx
        self.position.y += dy
        self.player_area.position = self.position
        self.suction_area.position = self.position

    def collect_balls(self, balls: List[CircleObject]) -> List[CircleObject]:
        for ball in balls:
            if self.suction_area.contains_point(ball.position):
                self.collected_balls.append(ball)
        return self.collected_balls
    
    @classmethod
    def create_player(cls, position: Position, width: int, height: int, degrees: float, suction_multiplier: float):
        player_area = SquareObject(position=position, width=width, height=height, degrees=degrees)
        suction_area = SquareObject(position=position, width=width * suction_multiplier, height=height * suction_multiplier, degrees=degrees)
        return cls(position=position, player_area=player_area, suction_area=suction_area, collected_balls=[])
    
class Wall(BaseModel):
    position: Position
    width: int
    height: int 