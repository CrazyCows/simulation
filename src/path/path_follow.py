from dto.robot import Robot, Move, MoveCommand, CheckpointType, RobotMode, Direction
from dto.shapes import CircleObject, SquareObject
from dto.obstacles import Cross, Wall
from typing import List
import math
import numpy as np


# TODO: Move this somewhere else. Idk where, but somewhere!
def create_move(robot: Robot) -> Move:
    """
    Moves the robot closer to the first checkpoint in the robots list of checkpoints.
    """
    radians = calculate_radians_to_turn(robot)  # We already calculated the checkpoint to go to elsewhere...
    speed = 1
    suck = suck_if_small(robot)
    if robot.mode == RobotMode.SAFE:
        if robot.is_robot_to_close_to_cross():
            t_speed, t_radians = avoid_cross(robot)
            if t_speed != -100:
                speed = t_speed
            if t_radians != -100:
                radians = t_radians
        else:
            speed = 2
    elif robot.mode == RobotMode.DANGER:
        if robot.mode == RobotMode.DANGER and radians != 0.0 and robot.prev_checkpoint.checkpoint_type == CheckpointType.DANGER_CHECKPOINT:
            speed = 0
        elif robot.mode == RobotMode.DANGER and radians == 0.0 and robot.prev_checkpoint.checkpoint_type == CheckpointType.DANGER_CHECKPOINT:
            speed = 1  # TODO: Implement logic for slowing down/stopping.
        else:
            speed = -1
    elif robot.mode == RobotMode.DANGER_REVERSE:
        speed = -1
        radians = 0

    move = Move(speed=speed, radians=radians, suck=suck)
    print(move)
    return move


# TODO: Move this somewhere else. Idk where, but somewhere!
def move_robot(move: Move, robot: Robot, walls: List[Wall], balls: List[CircleObject], cross: Cross,
               sim_only: bool = True):
    robot.move(move, walls, balls, cross)

    if sim_only is False:
        [balls.remove(ball) for ball in balls if ball in robot.collected_balls]
    if robot.self_reached_checkpoint(robot.checkpoints[0]):
        robot.prev_checkpoint = robot.checkpoints[0]
        if robot.prev_checkpoint.checkpoint_type == CheckpointType.DANGER_CHECKPOINT and robot.mode == RobotMode.SAFE:
            print(
                "Hallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\nHallo\n")
            robot.mode = RobotMode.DANGER
        elif robot.prev_checkpoint.checkpoint_type == CheckpointType.BALL and robot.mode == RobotMode.DANGER:
            robot.mode = RobotMode.DANGER_REVERSE
            robot.ignore_danger_in_corner = False
    if robot_distance_to_wall_min(robot) and robot.mode == RobotMode.DANGER_REVERSE:
        robot.ignore_danger_in_corner = False
        robot.mode = RobotMode.SAFE


def robot_distance_to_wall_min(robot: Robot):
    if robot.distance_to_wall_right > 100 and robot.distance_to_wall_left > 100 and robot.distance_to_wall_top > 100 and robot.distance_to_wall_top > 100:
        return True
    else:
        return False

def calculate_radians_to_turn(robot: Robot) -> float:
    """
    Auto-correcting algorithm which (hopefully) always keeps the robot looking
    towards the checkpoint.
    """
    # if robot.checkpoints != []:
    #    return robot
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
        for i in range(round(distance / 100)):
            direction += MoveCommand.LEFT.value
    elif cross_product > 0:
        direction += MoveCommand.RIGHT.value
        for i in range(round(distance / 100)):
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
    if distance_to_ball < 200:
        suck = True

    return suck

def avoid_cross(robot: Robot):
    speed = -100
    radians = -100

    actions = {
        Direction.NORTH: {
            Direction.NORTH: (speed, radians),
            Direction.SOUTH: (0.5, 0.025),
            Direction.EAST: (speed,-0.05),
            Direction.WEST: (speed, 0.05),
            Direction.NORTH_EAST: (speed, radians),
            Direction.NORTH_WEST: (speed, radians),
            Direction.SOUTH_EAST: (speed, -0.03),
            Direction.SOUTH_WEST: (speed, 0.03),
        },
        Direction.SOUTH: {
            Direction.NORTH: (0.5, 0.025),
            Direction.SOUTH: (speed, radians),
            Direction.EAST: (speed, -0.02),
            Direction.WEST: (speed, 0.05),
            Direction.NORTH_EAST: (speed, -0.05),
            Direction.NORTH_WEST: (speed, 0.05),
            Direction.SOUTH_EAST: (speed, radians),
            Direction.SOUTH_WEST: (speed, radians),
        },
        Direction.EAST: {
            Direction.NORTH: (speed, 0.05),
            Direction.SOUTH: (speed, -0.05),
            Direction.EAST: (speed, radians),
            Direction.WEST: (0.5, 0.025),
            Direction.NORTH_EAST: (speed, radians),
            Direction.NORTH_WEST: (speed, -0.05),
            Direction.SOUTH_EAST: (speed, radians),
            Direction.SOUTH_WEST: (speed, 0.05),
        },
        Direction.WEST: {
            Direction.NORTH: (speed, -0.05),
            Direction.SOUTH: (speed, 0.05),
            Direction.EAST: (1, 0.025),
            Direction.WEST: (speed, radians),
            Direction.NORTH_EAST: (speed, -0.05),
            Direction.NORTH_WEST: (speed, radians),
            Direction.SOUTH_EAST: (speed, 0.05),
            Direction.SOUTH_WEST: (speed, radians),
        }
    }

    if robot.direction in actions and robot.placement_of_cross in actions[robot.direction]:
        speed, radians = actions[robot.direction][robot.placement_of_cross]

    return speed, radians
