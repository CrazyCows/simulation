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

class Direction(Enum):
    NORTH = 1
    SOUTH = 2
    EAST = 3
    WEST = 4
    NORTH_WEST = 5
    NORTH_EAST = 6
    SOUTH_WEST = 7
    SOUTH_EAST = 8


class Move(BaseModel):
    speed: float
    radians: float
    suck: bool


class Paths(BaseModel):
    paths: List[Position]


class CheckpointType(Enum):
    BALL = 1
    SAFE_CHECKPOINT = 2
    DANGER_CHECKPOINT = 3
    DANGER_REVERSE_CHECKPOINT = 4
    GOAL = 5


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
    front_distance_to_cross: float = 0
    distance_to_wall_left_list: List[float] = [0,0,0,0]
    distance_to_wall_right_list: List[float] = [0,0,0,0]
    distance_to_wall_top_list: List[float] = [0,0,0,0]
    distance_to_wall_bot_list: List[float] = [0,0,0,0]
    distance_to_cross_list: List[float] = [0,0,0,0]
    placement_of_cross: Direction = Direction.NORTH
    ignore_danger_in_corner: bool = False
    direction: Direction = Direction.NORTH
    focused: bool = False

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

    def move(self, move: Move, obstacles: List[Wall] = [], balls: List[CircleObject] = [], cross: Cross = None):
        """
            Args:
                speed: speed from the robots current position in pixels
                radians: Delta in radians from the robots current direction
                obstacles: Objects the robot can't collide with
        """

        # print(move)
        speed = move.speed
        radians = move.radians
        suck = move.suck
        # print(speed)
        # print(radians)
        # print(suck)
        radians += self.robot.radians
        dy = math.cos(radians) * speed
        dx = math.sin(radians) * speed

        if self.obstacle_detection(obstacles=obstacles) or self.obstacle_detection(
                obstacles=[cross.square_1, cross.square_2]):
            radians = 0
            # NOTE: This is pretty jank.....
            dx = self.start_position.x - self.robot.position.x
            dy = self.start_position.y - self.robot.position.y
            self.mode = RobotMode.SAFE
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

    def self_reached_checkpoint(self, checkpoint: Checkpoint):
        if checkpoint.checkpoint_type == CheckpointType.DANGER_CHECKPOINT:
            return True if (
                    self.robot.position.x + 3 > checkpoint.x > self.robot.position.x - 3 and
                    self.robot.position.y + 3 > checkpoint.y > self.robot.position.y - 3) else False
        else:
            return circle_square_touch(CircleObject(radius=2, position=checkpoint), self.robot)

    @classmethod
    def create_robot(cls, position: Position, width: int, height: int, radians: float,
                     suction_width: int, suction_height: int, suction_offset_x: int = 0,
                     suction_offset_y: int = 0, checkpoints=[],
                     line=LineObject(start_pos=Position(x=0, y=0), end_pos=Position(x=0, y=0))):
        if not 0 <= radians <= 2 * math.pi:
            raise Exception("Number must be within range 0 to 2 * pi")

        robot = SquareObject.create_square(position=position,
                                           width=width,
                                           height=height,
                                           radians=radians)
        suction = SquareObject.create_square(position=position,
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
                   previous_path=previous_path, start_position=robot.position, checkpoints=checkpoints, line=line,
                   prev_checkpoint=Checkpoint(x=position.x, y=position.y,
                                              checkpoint_type=CheckpointType.SAFE_CHECKPOINT))

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
        distance = abs(A * px + B * py + C) / math.sqrt(A ** 2 + B ** 2)
        return distance

    def self_to_wall_distance(self, walls: List[Wall], cross: Cross):
        min_distance = float('inf')
        for wall in walls:
            min_distance = float('inf')
            if wall.placement == WallPlacement.LEFT:
                x1, y1 = wall.vertices[2]
                x2, y2 = wall.vertices[3]
                for vertex in self.robot.vertices:
                    distance = self._point_to_line_distance(vertex[0], vertex[1], x1, y1, x2, y2)
                    min_distance = min(min_distance, distance)
                self.distance_to_wall_left = min_distance
            elif wall.placement == WallPlacement.RIGHT:
                x1, y1 = wall.vertices[0]
                x2, y2 = wall.vertices[1]
                for vertex in self.robot.vertices:
                    distance = self._point_to_line_distance(vertex[0], vertex[1], x1, y1, x2, y2)
                    min_distance = min(min_distance, distance)
                self.distance_to_wall_right = min_distance
            elif wall.placement == WallPlacement.TOP:
                x1, y1 = wall.vertices[1]
                x2, y2 = wall.vertices[2]
                for vertex in self.robot.vertices:
                    distance = self._point_to_line_distance(vertex[0], vertex[1], x1, y1, x2, y2)
                    min_distance = min(min_distance, distance)
                self.distance_to_wall_top = min_distance
            elif wall.placement == WallPlacement.BOT:
                x1, y1 = wall.vertices[0]
                x2, y2 = wall.vertices[3]
                for vertex in self.robot.vertices:
                    distance = self._point_to_line_distance(vertex[0], vertex[1], x1, y1, x2, y2)
                    min_distance = min(min_distance, distance)
                self.distance_to_wall_bot = min_distance

        num_vertices = len(cross.square_1.vertices)
        min_distance = float('inf')
        front_min_distance = float('inf')
        for i in range(num_vertices):
            next_vertex_i = i+1
            if next_vertex_i == 4:
                next_vertex_i = 0
            for j in range(len(self.robot.vertices)):
                next_vertex_j = j + 1
                if next_vertex_j == 4:
                    next_vertex_j = 0
                #distance = self._point_to_line_distance(vertex[0], vertex[1], x1, y1, x2, y2)
                distance = self.segment_to_segment_distance(cross.square_1.vertices[i], cross.square_1.vertices[next_vertex_i], self.robot.vertices[j], self.robot.vertices[next_vertex_j])
                if j == 1 or j == 2:
                    front_min_distance = min(front_min_distance, distance)
                min_distance = min(min_distance, distance)
            self.front_distance_to_cross = front_min_distance
            self.distance_to_cross = min_distance
        num_vertices = len(cross.square_2.vertices)
        for i in range(num_vertices):
            next_vertex_i = i+1
            if next_vertex_i == 4:
                next_vertex_i = 0
            for j in range(len(self.robot.vertices)):
                next_vertex_j = j + 1
                if next_vertex_j == 4:
                    next_vertex_j = 0
                #distance = self._point_to_line_distance(vertex[0], vertex[1], x1, y1, x2, y2)
                distance = self.segment_to_segment_distance(cross.square_2.vertices[i], cross.square_2.vertices[next_vertex_i], self.robot.vertices[j], self.robot.vertices[next_vertex_j])
                if j == 1 or j == 2:
                    front_min_distance = min(front_min_distance, distance)
                min_distance = min(min_distance, distance)
            self.front_distance_to_cross = front_min_distance
            self.distance_to_cross = min_distance

    def calculate_degrees_between_robot_and_suction(self):
        delta_x = self.line.end_pos.x - self.line.start_pos.x
        delta_y = self.line.end_pos.y - self.line.start_pos.y


        degrees = math.degrees(math.atan2(delta_y,delta_x))
        if degrees < 0:
            degrees += 360
        return degrees
    def robot_direction(self):
        degrees = self.calculate_degrees_between_robot_and_suction()

        if 315 < degrees or degrees <= 45:
            self.direction = Direction.EAST
        if 45 < degrees <= 135:
            self.direction = Direction.SOUTH
        if 135 < degrees <= 225:
            self.direction = Direction.WEST
        if  225< degrees <= 315:
            self.direction = Direction.NORTH

    def is_robot_to_close_to_cross(self):
        if self.front_distance_to_cross < 90:
            return True
        return False

    def robot_angle_to_cross(self, cross: Cross):
        delta_x = self.robot.position.x - cross.square_1.position.x
        delta_y = self.robot.position.y - cross.square_1.position.y

        degrees = math.degrees(math.atan2(delta_y, delta_x))
        if degrees < 0:
            degrees += 360
        return degrees
    def robot_placement_of_cross(self, cross: Cross):
        degrees = self.robot_angle_to_cross(cross)

        if 337.5 < degrees or degrees <= 22.5:
            self.placement_of_cross = Direction.EAST
        elif 22.5 < degrees <= 67.5:
            self.placement_of_cross = Direction.SOUTH_EAST
        elif 67.5 < degrees <= 112.5:
            self.placement_of_cross = Direction.SOUTH
        elif 112.5 < degrees <= 157.5:
            self.placement_of_cross = Direction.SOUTH_WEST
        elif 157.5 < degrees <= 202.5:
            self.placement_of_cross = Direction.WEST
        elif 202.5 < degrees <= 247.5:
            self.placement_of_cross = Direction.NORTH_WEST
        elif 247.5 < degrees <= 292.5:
            self.placement_of_cross = Direction.NORTH
        elif 292.5 < degrees <= 337.5:
            self.placement_of_cross = Direction.NORTH_EAST
    def update_robot(self, cross: Cross):
        self.robot_placement_of_cross(cross)
        self.robot_direction()

    def is_robot_near_obstacles(self):
        threshold = 30
        if self.distance_to_cross> threshold:
            return False
        elif self.distance_to_wall_top > threshold:
            return False
        elif self.distance_to_wall_bot > threshold:
            return False
        elif self.distance_to_wall_left > threshold:
            return False
        elif self.distance_to_wall_right > threshold:
            return False
        return True

    def distance(self, p1, p2):
        """Beregn euklidisk afstand mellem to punkter."""
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


    def point_to_segment_distance(self, px, py, x1, y1, x2, y2):
        """Beregn afstanden fra et punkt til et linjesegment."""
        segment_length = self.distance((x1, y1), (x2, y2))
        if segment_length == 0:
            return self.distance((px, py), (x1, y1))

        # Projektion af punktet på linjesegmentet
        t = max(0, min(1, ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / (segment_length ** 2)))
        projection_x = x1 + t * (x2 - x1)
        projection_y = y1 + t * (y2 - y1)

        return self.distance((px, py), (projection_x, projection_y))


    def segment_to_segment_distance(self, vertex1, vertex2, vertex3, vertex4):
        """Beregn den mindste afstand mellem to linjesegmenter."""
        p1 = vertex1
        p2 = vertex2
        p3 = vertex3
        p4 = vertex4

        distances = [
            self.point_to_segment_distance(p1[0], p1[1], p3[0], p3[1], p4[0], p4[1]),
            self.point_to_segment_distance(p2[0], p2[1], p3[0], p3[1], p4[0], p4[1]),
            self.point_to_segment_distance(p3[0], p3[1], p1[0], p1[1], p2[0], p2[1]),
            self.point_to_segment_distance(p4[0], p4[1], p1[0], p1[1], p2[0], p2[1])
        ]

        return min(distances)