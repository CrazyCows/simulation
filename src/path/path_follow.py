from dto.robot import Robot, Move, MoveCommand
from dto.shapes import CircleObject, SquareObject
from dto.obstacles import Cross
from typing import List
from helper import overlap_detection
import math
import numpy as np

from src import Position
from src.dto.robot import Phase, CheckpointType, Direction


# TODO: Move this somewhere else. Idk where, but somewhere!
def create_move(robot: Robot, walls: list[SquareObject]) -> Move:
    """
    Moves the robot closer to the first checkpoint in the robots list of checkpoints.
    """

    robot_position = robot.robot.position
    if robot.checkpoints[0].checkpointType.value == CheckpointType.GOAL.value:
        goal = robot.checkpoints[0]
        if abs(robot_position.x - goal.x) < 20 and abs(robot_position.y - goal.y) < 20:
            print("YES")

            if abs(walls[0].position.x - robot_position.x) < abs(walls[1].position.x - robot_position.x):
                target_direction = Direction.WEST.value
            else:
                target_direction = Direction.EAST.value
            target_direction = Direction.NORTH.value
            radians = align_robot_to_direction(robot=robot, target_direction=target_direction)
            print("radians", radians)
            move = Move(speed=0.0, radians=radians, suck=False)
            return move

    speed = MoveCommand.FORWARD.value  # TODO: Implement logic for slowing down/stopping.
    radians = calculate_radians_to_turn(robot=robot, point_of_interest=Position(x=robot.checkpoints[0].x, y=robot.checkpoints[0].y))  # OLD COMMENT: We already calculated the checkpoint to go to elsewhere...
    suck = suck_if_small(robot)
    move = Move(speed=speed, radians=radians, suck=suck)

    return move


def align_robot_to_direction(robot: Robot, target_direction: str) -> float:
    """
    Aligns the robot to face a specified cardinal direction (north, east, south, west).
    """
    # Define target angles for each cardinal direction
    target_angles = {
        "west": math.pi / 2,
        "north": 0,
        "east": -math.pi / 2,
        "south": math.pi
    }

    # Get the current heading of the robot
    current_heading = robot.robot.radians

    # Get the target angle based on the specified direction
    if target_direction not in target_angles:
        raise ValueError("Invalid target direction. Must be one of: 'north', 'east', 'south', 'west'.")
    target_angle = target_angles[target_direction]

    # Calculate the angle difference
    angle_difference = target_angle - current_heading

    # Normalize the angle to be within [-pi, pi]
    angle_difference = (angle_difference + math.pi) % (2 * math.pi) - math.pi

    # Determine the direction to turn
    if angle_difference < 0:
        turn_direction = MoveCommand.LEFT.value
    elif angle_difference > 0:
        turn_direction = MoveCommand.RIGHT.value
    else:
        turn_direction = 0.0  # This happens when the point is directly on the line

    print(
        f"Current heading: {current_heading}, Target angle: {target_angle}, Angle difference: {angle_difference}, Turn direction: {turn_direction}")

    return turn_direction


# TODO: Move this somewhere else. Idk where, but somewhere!
def move_robot(move: Move, robot: Robot, obstacles: List[SquareObject], balls: List[CircleObject], cross: Cross, sim_only: bool = True):
    robot.move(move, obstacles, balls, cross)

    if sim_only is False:
        [balls.remove(ball) for ball in balls if ball in robot.collected_balls]
    if overlap_detection.circle_square_touch(CircleObject(radius=20, position=robot.checkpoints[0]), robot.robot):
        robot.prev_checkpoint = robot.checkpoints[0]


def calculate_radians_to_turn(robot: Robot, point_of_interest: Position) -> float:
    """
    Auto-correcting algorithm which (hopefully) always keeps the robot looking
    towards the checkpoint.
    """

    debug = False

    # Convert the inputs to numpy arrays for easier manipulation
    start_pos = (robot.line.start_pos.x, robot.line.start_pos.y)
    end_pos = (robot.line.end_pos.x, robot.line.end_pos.y)
    coordinate = (point_of_interest.x, point_of_interest.y)
    start = np.array(start_pos)
    end = np.array(end_pos)
    point = np.array(coordinate)

    if debug:
        print(f"Start position: {start_pos}, End position: {end_pos}, Point of interest: {coordinate}")

    # Calculate the line vector and the point vector
    line_vector = end - start
    point_vector = point - start
    if debug:
        print(f"Line vector: {line_vector}, Point vector: {point_vector}")

    # Calculate the projection of the point vector onto the line vector
    line_length_squared = np.dot(line_vector, line_vector)
    if line_length_squared == 0:
        if debug:
            print("Line length squared is zero")
        return 0.0  # The start and end positions are the same

    t = max(0, min(1, np.dot(point_vector, line_vector) / line_length_squared))

    # Calculate the nearest point on the line segment to the point
    nearest_point = start + t * line_vector

    if debug:
        print(f"Nearest point on the line: {nearest_point}")

    # Calculate the distance from the point to the nearest point on the line segment
    distance = np.linalg.norm(point - nearest_point)

    if debug:
        print(f"Distance from point to nearest point: {distance}")

    if distance < 5:
        if debug:
            print("Distance is less than 5, returning 0.0")
        return 0.0

    # Determine the direction to turn (left or right)
    # We can use the cross product of the line vector and point vector to determine the direction
    cross_product = np.cross(line_vector, point_vector)
    direction = 0.0
    if cross_product < 0:
        direction += MoveCommand.LEFT.value
        for i in range(round(distance / 150)):
            direction += MoveCommand.LEFT.value
    elif cross_product > 0:
        direction += MoveCommand.RIGHT.value
        for i in range(round(distance / 150)):
            direction += MoveCommand.RIGHT.value
    else:
        direction = 0.0  # This happens when the point is directly on the line

    print(f"Direction: {direction}")

    return direction

def suck_if_small(robot: Robot) -> bool:
    """
    Sucks if the distance to the ball is too small.
    """
    suck = False
    robot_pos = (robot.robot.position.x, robot.robot.position.y)
    distance_to_ball = math.dist(robot_pos, (robot.checkpoints[0].x, robot.checkpoints[0].y))
    if distance_to_ball < 25:
        suck = True

    return suck
