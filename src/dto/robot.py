from pydantic import BaseModel, confloat
from typing import Optional, List, Tuple
import math
import numpy as np
import logging
from enum import Enum
from helper.overlap_detection import square_touching, circle_square_touch, calculate_coordinates_for_line
from dto.shapes import SquareObject, CircleObject, LineObject, Position
from dto.obstacles import Cross, Wall, WallPlacement


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
    latch: bool = False
    

class Paths(BaseModel):
    paths: List[Position]

class CheckpointType(Enum):
    BALL = 1
    SAFE_CHECKPOINT = 2
    DANGER_CHECKPOINT = 3
    GOAL = 4

class Checkpoint(Position):
    checkpoint_type: CheckpointType = CheckpointType.SAFE_CHECKPOINT


class RobotMode(Enum):
    SAFE = 1
    DANGER = 2
    DANGER_REVERSE = 3

class Robot(BaseModel):
    robot: SquareObject
    suction: SquareObject
    collected_balls: List[CircleObject]
    obstacles_hit_list: List[SquareObject]
    previous_path: List[Position]
    prev_checkpoint: Checkpoint
    checkpoints: List[Checkpoint]
    start_position: Position
    line: LineObject
    mode: RobotMode = RobotMode.SAFE
    distance_to_wall_left: float = 0
    distance_to_wall_right: float = 0
    distance_to_wall_top: float = 0
    distance_to_wall_bot: float = 0
    distance_to_cross: float = 0

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
    
    def calculate_dist_to_checkpoint(self, checkpoint: Checkpoint):
        return math.dist((self.robot.position.x, self.robot.position.y), (checkpoint.x, checkpoint.y))
                

    def move(self, move: Move, obstacles: List[Wall]=[], balls: List[CircleObject]=[], cross: Cross = None):
        """
            Args:
                speed: speed from the robots current position in pixels
                radians: Delta in radians from the robots current direction
                obstacles: Objects the robot can't collide with
        """

        #print(move)
        speed = move.speed
        radians = move.radians
        suck = move.suck
        #print(speed)
        #print(radians)
        #print(suck)
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
        self.self_to_wall_distance(obstacles, cross)
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
                   previous_path=previous_path, start_position=robot.position, checkpoints=checkpoints, line=line, prev_checkpoint=Checkpoint(x=position.x, y=position.y, checkpoint_type=CheckpointType.SAFE_CHECKPOINT))

    def calculate_coordinates_for_line(direction, start_x, start_y, length=1200):
        """
            Calculates a straight line from coordiantes with a chosen length and angle.
        """

        
        end_x = start_x + length * math.sin(direction)
        end_y = start_y + length * math.cos(direction)
        
        return LineObject(start_pos=Position(x=start_x, y=start_y), end_pos=Position(x=end_x, y=end_y))


    def _point_to_line_distance(self, px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
        # Line equation coefficients A, B, and C
        A = y2 - y1
        B = x1 - x2
        C = x2 * y1 - x1 * y2
        
        # Distance from point (px, py) to the line (Ax + By + C = 0)
        distance = abs(A * px + B * py + C) / math.sqrt(A**2 + B**2)
        return distance

    def self_to_wall_distance(self, walls: List[Wall], cross: Cross):
        min_distance = float('inf')
        for wall in walls:
            num_vertices = len(wall.vertices)
            min_distance = float('inf')
            for i in range(num_vertices):
                x1, y1 = wall.vertices[i]
                x2, y2 = wall.vertices[(i + 1) % num_vertices]
                distance = self._point_to_line_distance(self.robot.position.x, self.robot.position.y, x1, y1, x2, y2)
                min_distance = min(min_distance, distance)
            if wall.placement == WallPlacement.LEFT:
                self.distance_to_wall_left = min_distance
            elif wall.placement == WallPlacement.RIGHT:
                self.distance_to_wall_right = min_distance
            elif wall.placement == WallPlacement.TOP:
                self.distance_to_wall_top = min_distance
            elif wall.placement == WallPlacement.BOT:
                self.distance_to_wall_bot = min_distance
        for wall in cross:
            min_distance = float('inf')
            num_vertices = len(wall.vertices)
            for i in range(num_vertices):
                x1, y1 = wall.vertices[i]
                x2, y2 = wall.vertices[(i + 1) % num_vertices]
                distance = self._point_to_line_distance(self.robot.position.x, self.robot.position.y, x1, y1, x2, y2)
                min_distance = min(min_distance, distance)
            self.distance_to_cross = min_distance
        


