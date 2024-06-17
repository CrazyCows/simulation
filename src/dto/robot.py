from pydantic import BaseModel, confloat
from typing import Optional, List, Tuple
import math
import numpy as np
import logging
from enum import Enum
from helper.overlap_detection import square_touching, circle_square_touch, calculate_coordinates_for_line
from dto.shapes import SquareObject, CircleObject, LineObject, Position
from dto.obstacles import Cross


class MoveCommand(Enum):
    FORWARD = 2.0
    BACKWARD = -2.0
    LEFT = 0.025
    RIGHT = -0.025
    SUCK = True

class Move(BaseModel):
    speed: float
    radians: float
    suck: bool
    

class Paths(BaseModel):
    paths: List[Position]

class Checkpoint(Position):
    is_ball: bool


class Robot(BaseModel):
    robot: SquareObject
    suction: SquareObject
    collected_balls: List[CircleObject]
    obstacles_hit_list: List[SquareObject]
    previous_path: List[Position]
    prev_checkpoint: Checkpoint = None
    checkpoints: List[Checkpoint]
    start_position: Position
    line: LineObject

    def suck(self, balls: List[CircleObject]):
        for ball in balls:
            if circle_square_touch(ball, self.suction):
                self.collected_balls.append(ball)

    def obstacle_detection(self, obstacles) -> bool:
        obstacle_touched = False
        for obstacle in obstacles:
            if square_touching(obstacle, self.robot):
                self.obstacles_hit_list.append(obstacle)
                obstacle_touched = True
        return obstacle_touched
                

    def move(self, move: Move, obstacles: List[SquareObject]=[], balls: List[CircleObject]=[], cross: Cross = None):
        """
            Args:
                speed: speed from the robots current position in pixels
                radians: Delta in radians from the robots current direction
                obstacles: Objects the robot can't collide with
        """

        print(self.robot.radians)
        print(move)
        speed = move.speed
        radians = move.radians
        suck = move.suck
        print(speed)
        print(radians)
        print(suck)
        radians += self.robot.radians
        dy = math.cos(radians) * speed
        dx = math.sin(radians) * speed

        if self.obstacle_detection(obstacles=obstacles) or self.obstacle_detection(obstacles=[cross.square_1, cross.square_2]):
            radians = 0
            # NOTE: This is pretty jank.....
            dx = self.start_position.x - self.robot.position.x
            dy = self.start_position.y - self.robot.position.y
            logging.warning('robot touched a wall - resetting position')

        robot_dx = self.robot.position.x + dx
        robot_dy = self.robot.position.y + dy
        self.previous_path.append(self.robot.position)
        self.robot.update_square(Position(x=robot_dx, y=robot_dy), radians)
        self.suction.update_square(Position(x=robot_dx, y=robot_dy), radians)
        if suck:
            self.suck(balls=balls)
        self.line = calculate_coordinates_for_line(
            radians, self.robot.position.x, self.robot.position.y
        )

    def set_robot(self, position: Position, radians: float):
        self.robot.position = position
        self.robot.radians = radians

    def calculate_speed_to_ball(self, ball: CircleObject):
        return math.dist((self.robot.position.x, self.robot.position.y), (ball.position.x, ball.position.y))

    @classmethod
    def create_robot(cls, position: Position, width: int, height: int, radians: float, 
                suction_width: int, suction_height: int, suction_offset_x: int=0, 
                suction_offset_y: int=0, checkpoints=[], line=LineObject(start_pos=Position(x=0, y=0), end_pos=Position(x=0, y=0))):
        if not 0 <= radians <= 2 * math.pi:
            raise Exception("Number must be within range 0 to 2 * pi")

        robot=SquareObject.create_square(position=position, 
                                 width=width, 
                                 height=height, 
                                 radians=radians)
        suction=SquareObject.create_square(position=position, 
                                  width=suction_width, 
                                  height=suction_height, 
                                  radians=radians,
                                  offset_x=suction_offset_x,
                                  offset_y=suction_offset_y)
        collected_balls = []
        
        obstacles_hit_list = []
        obstacles_hit = 0
        previous_path = []
        
        return cls(robot=robot, suction=suction, collected_balls=collected_balls, 
                   obstacles_hit_list=obstacles_hit_list, obstacles_hit=obstacles_hit, 
                   previous_path=previous_path, start_position=robot.position, checkpoints=checkpoints, line=line)