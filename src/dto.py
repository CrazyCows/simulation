from pydantic import BaseModel, confloat
from typing import Optional, List, Tuple
import math
import numpy as np
import logging
from enum import Enum

class Input(Enum):
    FORWARD = 2
    BACKWARD = -2
    LEFT = 0.01
    RIGHT = -0.01
    SUCK = True


class Inputs(BaseModel):
    inputs: List[Input]
    
    def read_inputs(self):
        d_forward = 0
        d_radians = 0
        suck = False

        for input in self.inputs:
            if input == Input.FORWARD:
                d_forward += Input.FORWARD.value
            elif input == Input.BACKWARD:
                d_forward -= Input.BACKWARD.value
            elif input == Input.LEFT:
                d_radians += Input.LEFT.value
            elif input == Input.RIGHT:
                d_radians -= Input.RIGHT.value
            elif input == Input.SUCK:
                suck = True
        return d_forward, d_radians, suck

class RewardValues(Enum):
    BALL_COLLECT = 10
    WALL_HIT = 10
    MOVING_FORWARD = 0.01
    MOVING_SIDEWAYS = -0.05
    SUCK = -0.01

class Reward(BaseModel):
    balls_collected: float = 0
    walls_hit: float = 0
    points_for_moving_forward: float = 0
    points_for_moving_sideways: float = 0
    points_for_suck: float = 0

class Position(BaseModel):
    x: float
    y: float

class Paths(BaseModel):
    paths: List[Position]

class Checkpoint(Position):
    is_ball: bool

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

    def update_square(self, position: Position, radians: float = 0):
        # Apply offset to the position
        position_with_offset = Position(x=position.x + self.offset_x, y=position.y + self.offset_y)
        self.position = position_with_offset
        print(f'new pso: {position.y}')
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
    def create_square(cls, position: Position, width: int, height: int, radians: float, offset_x=0, offset_y=0):
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
                   vertices=vertices)

class CircleObject(BaseModel):
    radius: int
    position: Position

    def contains_point(self, point: Position) -> bool:
        return math.hypot(self.position.x - point.x, self.position.y - point.y) <= self.radius
    

class Player(BaseModel):
    player: SquareObject
    suction: SquareObject
    collected_balls: List[CircleObject]
    obstacles_hit_list: List[SquareObject]
    previous_path: List[Position]
    checkpoints: List[Checkpoint]
    start_position: Position
    rewards: Reward
    line: LineObject

    def suck(self, balls: List[CircleObject]):
        for ball in balls:
            if circle_square_touch(ball, self.suction):
                self.collected_balls.append(ball)

    def obstacle_detection(self, obstacles) -> bool:
        obstacle_touched = False
        for obstacle in obstacles:
            if square_touching(obstacle, self.player):
                self.obstacles_hit_list.append(obstacle)
                obstacle_touched = True
        return obstacle_touched
                

    def move(self, inputs: Inputs, obstacles: List[SquareObject]=[], balls: List[CircleObject]=[]):
        """
            Args:
                distance: Distance from the players current position in pixels
                radians: Delta in radians from the players current direction
                obstacles: Objects the player can't collide with
        """

        distance, radians, suck = inputs.read_inputs()

        radians += self.player.radians
        dy = math.cos(radians) * distance
        dx = math.sin(radians) * distance

        if self.obstacle_detection(obstacles=obstacles):
            radians = 0
            # NOTE: This is pretty jank.....
            dx = self.start_position.x - self.player.position.x
            dy = self.start_position.y - self.player.position.y
            logging.warning('Player touched a wall - resetting position')

        player_dx = self.player.position.x + dx
        player_dy = self.player.position.y + dy
        self.previous_path.append(self.player.position)
        self.player.update_square(Position(x=player_dx, y=player_dy), radians)
        self.suction.update_square(Position(x=player_dx, y=player_dy), radians)
        if suck:
            self.suck(balls=balls)
        self.rewards.balls_collected = len(self.collected_balls)
        self.rewards.walls_hit = len(self.obstacles_hit_list)
        self.line = calculate_coordinates_for_line(
            radians, self.player.position.x, self.player.position.y
        )

    def set_player(self, position: Position, radians: float):
        self.player.position = position
        self.player.radians = radians

    
        


    @classmethod
    def create_player(cls, position: Position, width: int, height: int, radians: float, 
                suction_width: int, suction_height: int, suction_offset_x: int=0, 
                suction_offset_y: int=0, checkpoints=[], line=LineObject(start_pos=Position(x=0, y=0), end_pos=Position(x=0, y=0))):
        if not 0 <= radians <= 2 * math.pi:
            raise Exception("Number must be within range 0 to 2 * pi")

        player=SquareObject.create_square(position=position, 
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
        rewards = Reward(balls_collected=0, walls_hit=0, points_for_moving_forward=0, points_for_moving_sideways=0, points_for_suck=0)
        return cls(player=player, suction=suction, collected_balls=collected_balls, 
                   obstacles_hit_list=obstacles_hit_list, obstacles_hit=obstacles_hit, 
                   previous_path=previous_path, start_position=player.position, rewards=rewards, checkpoints=checkpoints, line=line)

# TODO: Move everything below this line into more suitable locations if time allows.
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

def square_touching(square1: SquareObject, square2: SquareObject) -> bool:
    def project(vertices, axis):
        dots = [np.dot(vertex, axis) for vertex in vertices]
        return [min(dots), max(dots)]

    def overlap(proj1, proj2):
        # Adjust to check for exact touching or minimal overlap
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
    distance = math.hypot(circle1.position.x - circle2.position.x, circle1.position.y - circle2.position.y)
    radius_sum = circle1.radius + circle2.radius
    tolerance = 0.0001  # Small tolerance to handle floating-point precision issues
    return abs(distance - radius_sum) <= tolerance


def circle_square_touch(circle: CircleObject, square: SquareObject) -> bool:
    def circle_line_segment_touch(circle: CircleObject, p1: Position, p2: Position) -> bool:
        # Adjusted for exact touch
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

    # Check if any of the square's edges touch the circle
    for i in range(len(square.vertices)):
        p1 = Position(x=square.vertices[i][0], y=square.vertices[i][1])
        p2 = Position(x=square.vertices[(i + 1) % len(square.vertices)][0], y=square.vertices[(i + 1) % len(square.vertices)][1])
        if circle_line_segment_touch(circle, p1, p2):
            return True

    return False

def calculate_coordinates_for_line(direction, start_x, start_y, length=1200):
    # Calculate the ending coordinates
    end_x = start_x + length * math.sin(direction)
    end_y = start_y + length * math.cos(direction)
    
    # Return the beginning and ending coordinates as tuples
    return LineObject(start_pos=Position(x=start_x, y=start_y), end_pos=Position(x=end_x, y=end_y))