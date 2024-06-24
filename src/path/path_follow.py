from dto.robot import Robot, Move, MoveCommand, CheckpointType, RobotMode, Position
from dto.shapes import CircleObject, SquareObject
from dto.obstacles import Cross, Wall
from typing import List
from helper import overlap_detection
import math
import numpy as np


# TODO: Move this somewhere else. Idk where, but somewhere!
def create_move(robot: Robot) -> Move:
    """
    Moves the robot closer to the first checkpoint in the robots list of checkpoints.
    """
    radians = calculate_radians_to_turn(robot)  # We already calculated the checkpoint to go to elsewhere...
    speed = 0
    latch = False
    if robot.mode == RobotMode.DANGER or robot.mode == RobotMode.ENDPHASE:
        if radians != 0.0 and (
                robot.prev_checkpoint.checkpoint_type == CheckpointType.DANGER_CHECKPOINT or
                robot.prev_checkpoint.checkpoint_type == CheckpointType.GOAL_LINEUP or
                robot.prev_checkpoint.checkpoint_type == CheckpointType.BALL or
                robot.prev_checkpoint.checkpoint_type == CheckpointType.SAFE_CHECKPOINT):
            speed = 0
        elif radians == 0.0 and (
                robot.prev_checkpoint.checkpoint_type == CheckpointType.DANGER_CHECKPOINT or
                robot.prev_checkpoint.checkpoint_type == CheckpointType.GOAL_LINEUP or
                robot.prev_checkpoint.checkpoint_type == CheckpointType.BALL or
                robot.prev_checkpoint.checkpoint_type == CheckpointType.SAFE_CHECKPOINT):
            speed = 1  # TODO: Implement logic for slowing down/stopping.
    elif robot.mode == RobotMode.SAFE:
        if robot.calculate_dist_to_checkpoint(robot.checkpoints[0]) > 150:
            speed = 1
        elif robot.calculate_dist_to_checkpoint(robot.checkpoints[0]) < 150 and radians != 0:
            speed = 0
        # Tilføjet så roboten stopper foran bolden
        # TODO: Måde at håndtere hvis roboten ikke får bolden op
        elif robot.calculate_dist_to_checkpoint(robot.checkpoints[0]) < 150 and radians == 0 and robot.checkpoints[0].checkpoint_type == CheckpointType.BALL:
            speed = 0
        else:
            speed = MoveCommand.FORWARD.value  # TODO: Implement logic for slowing down/stopping.
    elif robot.mode == RobotMode.DANGER_REVERSE:
        speed = -1
        radians = 0
    if robot.mode == RobotMode.DEPOSIT:
        speed = 0
        radians = 0
        latch = True
    if robot.mode == RobotMode.STOP_DANGER or robot.mode == RobotMode.STOP:
        speed = 0
        radians = 0

    suck = False if robot.mode == RobotMode.DEPOSIT else suck_if_small(robot)
    move = Move(speed=speed, radians=radians, suck=suck, latch=latch)
    #print(move)
    return move


# TODO: Move this somewhere else. Idk where, but somewhere!
def move_robot(move: Move, robot: Robot, walls: List[Wall], balls: List[CircleObject], cross: Cross,
               sim_only: bool = True):
    robot.move(move, walls, balls, cross)
    if sim_only is False:
        [balls.remove(ball) for ball in balls if ball in robot.collected_balls]

        if balls == []:

            return
            exit()
    if robot.self_reached_checkpoint(robot.checkpoints[0]):
        #print("Previous checkpoint before being updated: ", robot.prev_checkpoint)
        robot.prev_checkpoint = robot.checkpoints[0]
        if robot.prev_checkpoint.checkpoint_type == CheckpointType.DANGER_CHECKPOINT and robot.mode == RobotMode.SAFE:
            robot.mode = RobotMode.DANGER
        elif robot.prev_checkpoint.checkpoint_type == CheckpointType.BALL:
            if robot.mode == RobotMode.SAFE:
                robot.mode = RobotMode.STOP
            elif robot.mode == RobotMode.DANGER:
                robot.mode = RobotMode.STOP_DANGER
                robot.ignore_danger_in_corner = False
            elif robot.mode == RobotMode.STOP_DANGER:
                robot.mode = RobotMode.DANGER_REVERSE
            elif robot.mode == RobotMode.STOP:
                robot.mode = RobotMode.SAFE
        elif robot.prev_checkpoint.checkpoint_type == CheckpointType.GOAL:
            if robot.mode == RobotMode.ENDPHASE or robot.mode == RobotMode.DEPOSIT:
                robot.mode = RobotMode.DEPOSIT
    if not robot.is_robot_near_obstacles(50) and robot.mode == RobotMode.DANGER_REVERSE:
        robot.ignore_danger_in_corner = False
        robot.mode = RobotMode.SAFE


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
    if (distance_to_ball < 100 and robot.ignore_danger_in_corner == False) or (
            robot.ignore_danger_in_corner and distance_to_ball < 200):
        suck = True

    return suck