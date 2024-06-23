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
    latch: bool


class Paths(BaseModel):
    paths: List[Position]


class CheckpointType(Enum):
    BALL = 1
    SAFE_CHECKPOINT = 2
    DANGER_CHECKPOINT = 3
    DANGER_REVERSE_CHECKPOINT = 4
    GOAL = 5
    GOAL_LINEUP = 6


class Checkpoint(Position):
    checkpoint_type: CheckpointType = CheckpointType.SAFE_CHECKPOINT


class RobotMode(Enum):
    SAFE = 1
    DANGER = 2
    DANGER_REVERSE = 3
    STOP = 4
    STOP_DANGER = 5
    DEPOSIT = 6
    ENDPHASE = 7


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
    ignore_danger_in_corner: bool = False

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
        if (checkpoint.checkpoint_type == CheckpointType.DANGER_CHECKPOINT or
                checkpoint.checkpoint_type == CheckpointType.SAFE_CHECKPOINT or
                checkpoint.checkpoint_type == CheckpointType.GOAL_LINEUP):
            return True if (
                    self.robot.position.x + 4 > checkpoint.x > self.robot.position.x - 4 and
                    self.robot.position.y + 4 > checkpoint.y > self.robot.position.y - 4) else False
        else:
            return circle_square_touch(CircleObject(radius=10, position=checkpoint), self.suction)

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
        cls.line = calculate_coordinates_for_line(
            radians, position.x, position.y
        )

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

    def is_robot_near_obstacles(self, threshold: float = 30):
        if self.distance_to_cross < threshold:
            return True
        elif self.distance_to_wall_top < threshold:
            return True
        elif self.distance_to_wall_bot < threshold:
            return True
        elif self.distance_to_wall_left < threshold:
            return True
        elif self.distance_to_wall_right < threshold:
            return True
        return False

    def self_to_wall_distance(self, walls: List[Wall], cross: Cross):
        min_distance = float('inf')
        for wall in walls:
            min_distance = float('inf')
            if wall.placement == WallPlacement.LEFT:
                v1 = wall.vertices[2]
                v2 = wall.vertices[3]
                for j in range(len(self.robot.vertices)):
                    next_vertex_j = j + 1
                    if next_vertex_j == 4:
                        next_vertex_j = 0
                    # distance = self._point_to_line_distance(vertex[0], vertex[1], x1, y1, x2, y2)
                    distance = self.segment_to_segment_distance(v1,
                                                                v2,
                                                                self.robot.vertices[j],
                                                                self.robot.vertices[next_vertex_j])
                    min_distance = min(min_distance, distance)
                self.distance_to_wall_left = min_distance
            elif wall.placement == WallPlacement.RIGHT:
                v1 = wall.vertices[0]
                v2 = wall.vertices[1]
                for j in range(len(self.robot.vertices)):
                    next_vertex_j = j + 1
                    if next_vertex_j == 4:
                        next_vertex_j = 0
                    # distance = self._point_to_line_distance(vertex[0], vertex[1], x1, y1, x2, y2)
                    distance = self.segment_to_segment_distance(v1,
                                                                v2,
                                                                self.robot.vertices[j],
                                                                self.robot.vertices[next_vertex_j])
                    min_distance = min(min_distance, distance)
                self.distance_to_wall_right = min_distance
            elif wall.placement == WallPlacement.TOP:
                v1 = wall.vertices[1]
                v2 = wall.vertices[2]
                for j in range(len(self.robot.vertices)):
                    next_vertex_j = j + 1
                    if next_vertex_j == 4:
                        next_vertex_j = 0
                    # distance = self._point_to_line_distance(vertex[0], vertex[1], x1, y1, x2, y2)
                    distance = self.segment_to_segment_distance(v1,
                                                                v2,
                                                                self.robot.vertices[j],
                                                                self.robot.vertices[next_vertex_j])
                    min_distance = min(min_distance, distance)
                self.distance_to_wall_top = min_distance
            elif wall.placement == WallPlacement.BOT:
                v1 = wall.vertices[0]
                v2 = wall.vertices[3]
                for j in range(len(self.robot.vertices)):
                    next_vertex_j = j + 1
                    if next_vertex_j == 4:
                        next_vertex_j = 0
                    # distance = self._point_to_line_distance(vertex[0], vertex[1], x1, y1, x2, y2)
                    distance = self.segment_to_segment_distance(v1,
                                                                v2,
                                                                self.robot.vertices[j],
                                                                self.robot.vertices[next_vertex_j])
                    min_distance = min(min_distance, distance)
                self.distance_to_wall_bot = min_distance

        num_vertices = len(cross.square_1.vertices)
        min_distance = float('inf')
        front_min_distance = float('inf')
        for i in range(num_vertices):
            next_vertex_i = i + 1
            if next_vertex_i == 4:
                next_vertex_i = 0
            for j in range(len(self.robot.vertices)):
                next_vertex_j = j + 1
                if next_vertex_j == 4:
                    next_vertex_j = 0
                # distance = self._point_to_line_distance(vertex[0], vertex[1], x1, y1, x2, y2)
                distance = self.segment_to_segment_distance(cross.square_1.vertices[i],
                                                            cross.square_1.vertices[next_vertex_i],
                                                            self.robot.vertices[j], self.robot.vertices[next_vertex_j])
                if j == 1 or j == 2:
                    front_min_distance = min(front_min_distance, distance)
                min_distance = min(min_distance, distance)
            self.front_distance_to_cross = front_min_distance
            self.distance_to_cross = min_distance
        num_vertices = len(cross.square_2.vertices)
        for i in range(num_vertices):
            next_vertex_i = i + 1
            if next_vertex_i == 4:
                next_vertex_i = 0
            for j in range(len(self.robot.vertices)):
                next_vertex_j = j + 1
                if next_vertex_j == 4:
                    next_vertex_j = 0
                # distance = self._point_to_line_distance(vertex[0], vertex[1], x1, y1, x2, y2)
                distance = self.segment_to_segment_distance(cross.square_2.vertices[i],
                                                            cross.square_2.vertices[next_vertex_i],
                                                            self.robot.vertices[j], self.robot.vertices[next_vertex_j])
                if j == 1 or j == 2:
                    front_min_distance = min(front_min_distance, distance)
                min_distance = min(min_distance, distance)
            self.front_distance_to_cross = front_min_distance
            self.distance_to_cross = min_distance

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
