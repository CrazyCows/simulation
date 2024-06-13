from typing import List, Tuple
from dto.shapes import CircleObject, SquareObject, Position
from dto.robot import Robot, Checkpoint
from pydantic import BaseModel
import math

# TODO: Implement moving around obstacles
def create_path(balls: List[CircleObject], robot: Robot, obstacles: List[SquareObject]) -> Tuple[List[SquareObject], List[Checkpoint]]:
    """
        Creates checkpoints to follow by creating a path between all balls to pick up.

        args:
            List[SquareObject]: Creates a list of square objects mimicking the robots path, including width considerations. 
            List[Checkpoint]: List of checkpoints which the robot must pass.
    """

    temp_balls = sorted(balls, key=lambda ball: robot.calculate_speed_to_ball(ball))
    first_ball = True
    checkpoints = []  # A list of checkpoints the robot must pass
    path: List[SquareObject] = [] # Path is a line with a width to ensure the robot does not touch the obstacles
    def calculate_speed_to_ball(ball_start: CircleObject, ball_end: CircleObject):
        return math.dist((ball_start.position.x, ball_start.position.y), (ball_end.position.x, ball_end.position.y))

    while temp_balls:
        if first_ball:
            # Calculate the difference in x and y coordinates
            dx = robot.robot.position.x - temp_balls[0].position.x
            dy = robot.robot.position.y - temp_balls[0].position.y

            # Calculate the angle in radians
            angle_radians = math.atan2(dy, dx)
            height = round(math.dist((robot.robot.position.x, robot.robot.position.y), (temp_balls[0].position.x, temp_balls[0].position.y)))
            width = 30

            # Calculate the midpoint position between the robot and the first ball
            mid_x = (robot.robot.position.x + temp_balls[0].position.x) / 2
            mid_y = (robot.robot.position.y + temp_balls[0].position.y) / 2

            # Use this angle and midpoint position in your create_square function call
            temp_path = SquareObject.create_square(
                position=Position(x=mid_x, y=mid_y),
                width=width,
                height=height,
                radians=-(angle_radians - math.pi/2)
            )

            first_ball = False
        else:
            sort_ball = temp_balls[0]
            temp_balls.sort(key=lambda ball: calculate_speed_to_ball(ball, sort_ball))
            # Calculate the difference in x and y coordinates
            dx = temp_balls[0].position.x - temp_balls[1].position.x
            dy = temp_balls[0].position.y - temp_balls[1].position.y
            angle_radians = math.atan2(dy, dx)

            # Calculate the midpoint position between the first and second ball
            mid_x = (temp_balls[0].position.x + temp_balls[1].position.x) / 2
            mid_y = (temp_balls[0].position.y + temp_balls[1].position.y) / 2

            # Use this angle and midpoint position in your create_square function call
            temp_path = SquareObject.create_square(
                position=Position(x=mid_x, y=mid_y),
                width=30,
                height=round(math.dist((temp_balls[0].position.x, temp_balls[0].position.y), (temp_balls[1].position.x, temp_balls[1].position.y))),
                radians=-(angle_radians - math.pi/2)
            )
            temp_balls.pop(0)
        

        path.append(temp_path)  
        checkpoints.append(Checkpoint(x=temp_balls[0].position.x, y=temp_balls[0].position.y, is_ball=True))
        if len(temp_balls) == 1:
            temp_balls = None
        
    return path, checkpoints


def check_for_obstacles_in_path_and_recalculate():

    """"""


def check_for_obstacles():
    ""

