from dto.robot import Robot, Move, MoveCommand
from dto.shapes import CircleObject, SquareObject
from dto.obstacles import Cross
from typing import List
import math
import numpy as np

# TODO: Move this somewhere else. Idk where, but somewhere!
def create_move(robot: Robot) -> Move:
    """
    Moves the robot closer to the first checkpoint in the robots list of checkpoints.
    """
    speed = MoveCommand.FORWARD.value  # TODO: Implement logic for slowing down/stopping.
    radians = calculate_radians_to_turn(robot)  # We already calculated the checkpoint to go to elsewhere...
    suck = suck_if_small(robot)
    move = Move(speed=speed, radians=radians, suck=suck)
    
    return move

# TODO: Move this somewhere else. Idk where, but somewhere!
def move_robot(move: Move, robot: Robot, obstacles: List[SquareObject], balls: List[CircleObject], cross: Cross, sim_only: bool = True):
    robot.move(move, obstacles, balls, cross)

    if sim_only is False:
        [balls.remove(ball) for ball in balls if ball in robot.collected_balls]

def calculate_radians_to_turn(robot: Robot) -> float:
    """
    Auto-correcting algorithm which (hopefully) always keeps the robot looking
    towards the checkpoint.
    """

    # Convert the inputs to numpy arrays for easier manipulation
    start_pos = (robot.line.start_pos.x, robot.line.start_pos.y)
    end_pos = (robot.line.end_pos.x, robot.line.end_pos.y)
    coordinate = (robot.checkpoints[0].x, robot.checkpoints[0].y)
    start = np.array(start_pos)
    end = np.array(end_pos)
    point = np.array(coordinate)
    
    # Calculate the line vector and the point vector
    line_vector = end - start
    point_vector = point - start
    
    # Calculate the projection of the point vector onto the line vector
    line_length_squared = np.dot(line_vector, line_vector)
    if line_length_squared == 0:
        return 0.0  # The start and end positions are the same
    
    t = max(0, min(1, np.dot(point_vector, line_vector) / line_length_squared))
    
    # Calculate the nearest point on the line segment to the point
    nearest_point = start + t * line_vector
    
    # Calculate the distance from the point to the nearest point on the line segment
    distance = np.linalg.norm(point - nearest_point)
    if distance < 5:
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

def remove_checkpoint_if_reached(robot: Robot):
    """
    Removes the current checkpoint if it has been reached.
    """
    checkpoint_pos = (robot.checkpoints[0].x, robot.checkpoints[0].y)
    robot_pos = (robot.robot.position.x, robot.robot.position.y)
    distance_to_checkpoint = math.dist(robot_pos, checkpoint_pos)
    if distance_to_checkpoint < 15:
        robot.checkpoints.pop(0)
