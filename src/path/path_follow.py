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
    #print("rad_________________________________________________", radians)
    #print("my checkpOINTS:", robot.checkpoints)
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
        if robot.calculate_dist_to_checkpoint(robot.checkpoints[0]) > 20:
            speed = 1
        elif robot.calculate_dist_to_checkpoint(robot.checkpoints[0]) < 20 and radians != 0:
            speed = 0
            #print("robot is not moving 1")
        # Tilføjet så roboten stopper foran bolden
        # TODO: Måde at håndtere hvis roboten ikke får bolden op
        elif robot.calculate_dist_to_checkpoint(robot.checkpoints[0]) < 180 and abs(radians) < 0.15 and robot.checkpoints[0].checkpoint_type == CheckpointType.BALL:
            #print("robot is not moving 2")
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
        print("Previous checkpoint before being updated: ", robot.prev_checkpoint)
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


def another_calculate_radians_to_turn(robot: Robot) -> float:
    """
    Calculates the angle in radians that the robot needs to turn to face the checkpoint.
    """
    if not robot.checkpoints:
        return 0.0  # No checkpoints to navigate to

    # Get the current position of the robot and the position of the checkpoint
    current_pos = np.array([robot.line.end_pos.x, robot.line.end_pos.y])
    checkpoint_pos = np.array([robot.checkpoints[0].x, robot.checkpoints[0].y])

    # Calculate the vector from the robot to the checkpoint
    direction_vector = checkpoint_pos - current_pos

    # Calculate the angle to the checkpoint relative to the x-axis
    angle_to_checkpoint = np.arctan2(direction_vector[1], direction_vector[0])

    # Get the robot's current orientation (assuming robot.orientation is the angle in radians)
    current_orientation = robot.robot.radians

    # Calculate the angle difference
    angle_difference = angle_to_checkpoint - current_orientation

    # Normalize the angle to be within the range -pi to pi
    angle_difference = (angle_difference + np.pi) % (2 * np.pi) - np.pi

    return angle_difference


def yet_another_calculate_radians_to_turn(robot: Robot) -> float:
    """
    Calculates the angle in radians that the robot needs to turn to face the checkpoint.
    """
    if not robot.checkpoints:
        return 0.0  # No checkpoints to navigate to

    # Get the current position of the robot and the position of the checkpoint
    current_pos = np.array([robot.line.end_pos.x, robot.line.end_pos.y])
    checkpoint_pos = np.array([robot.checkpoints[0].x, robot.checkpoints[0].y])

    # Calculate the vector from the robot to the checkpoint
    direction_vector = checkpoint_pos - current_pos

    # Calculate the angle to the checkpoint relative to the x-axis
    angle_to_checkpoint = np.arctan2(direction_vector[1], direction_vector[0])

    # Get the robot's current orientation in radians
    current_orientation = robot.robot.radians

    # Calculate the angle difference
    angle_difference = angle_to_checkpoint - current_orientation

    # Normalize the angle to be within the range -pi to pi
    angle_difference = (angle_difference + np.pi) % (2 * np.pi) - np.pi

    direction = 0.0
    #print("angle difference calculated:", angle_difference)
    """if angle_difference < 0:
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

    #angle_difference += 0.39"""

    return angle_difference


def yet_again_another_calculate_radians_to_turn(robot: Robot) -> float:
    """
    Auto-correcting algorithm which (hopefully) always keeps the robot looking
    towards the checkpoint.
    """
    if not robot.checkpoints:
        return 0.0

    # Convert the inputs to numpy arrays for easier manipulation
    start_pos = np.array([robot.line.start_pos.x, robot.line.start_pos.y])
    end_pos = np.array([robot.line.end_pos.x, robot.line.end_pos.y])
    checkpoint = np.array([robot.checkpoints[0].x, robot.checkpoints[0].y])

    # Calculate the line vector and the vector to the checkpoint
    line_vector = end_pos - start_pos
    point_vector = checkpoint - start_pos

    # Get the robot's current orientation in radians
    current_orientation = robot.robot.radians

    # Calculate the vector representing the robot's current orientation
    robot_direction_vector = np.array([np.cos(current_orientation), np.sin(current_orientation)])

    # Normalize the vectors
    line_vector_normalized = line_vector / np.linalg.norm(line_vector)
    point_vector_normalized = point_vector / np.linalg.norm(point_vector)

    # Calculate the angle between the robot's current direction and the vector to the checkpoint
    dot_product = np.dot(robot_direction_vector, point_vector_normalized)
    cross_product = np.cross(robot_direction_vector, point_vector_normalized)
    angle = np.arctan2(cross_product, dot_product)

    # Determine the direction to turn (left or right)
    if angle < 0:
        direction = MoveCommand.LEFT.value
    else:
        direction = MoveCommand.RIGHT.value

    #print("NEW FUNC SAYS:", direction * abs(angle))

    return direction * abs(angle)  # Scale direction by angle magnitude


def calculate_radians_to_turn(robot: Robot) -> float:
    """
    Auto-correcting algorithm which (hopefully) always keeps the robot looking
    towards the checkpoint.
    """
    # if robot.checkpoints != []:
    #    return robot
    # Convert the inputs to numpy arrays for easier manipulation


    length = 1200
    start_x = robot.robot.position.x
    start_y = robot.robot.position.y
    end_x = start_x + length * math.sin(robot.robot.radians)
    end_y = start_y + length * math.cos(robot.robot.radians)

    start_pos = (start_x, start_y)
    end_pos = (end_x, end_y)
    coordinate = (robot.checkpoints[0].x, robot.checkpoints[0].y)
    start = np.array(start_pos)
    end = np.array(end_pos)
    point = np.array(coordinate)

    # prints all above
    #print("start_pos:", start_pos)
    #print("end_pos:", end_pos)
    #print("coordinate:", coordinate)
    #print("start:", start)
    #print("end:", end)
    #print("point:", point)


    # Calculate the line vector and the point vector
    line_vector = end - start
    point_vector = point - start

    # Calculate the projection of the point vector onto the line vector
    line_length_squared = np.dot(line_vector, line_vector)
    if line_length_squared == 0:
        #print("line length is 0_______________________")
        ##return 0.0  # The start and end positions are the same
        ""
    t = max(0, min(1, np.dot(point_vector, line_vector) / line_length_squared))

    # Calculate the nearest point on the line segment to the point
    nearest_point = start + t * line_vector

    # Calculate the distance from the point to the nearest point on the line segment
    distance = np.linalg.norm(point - nearest_point)
    if distance < 5:
        ""
        #print("distance is less than 5")
        #return 0.0

    # Determine the direction to turn (left or right)
    # We can use the cross product of the line vector and point vector to determine the direction
    cross_product = np.cross(line_vector, point_vector)
    dot_product = np.dot(line_vector, point_vector)
    angle = np.arctan2(cross_product, dot_product)  # Calculate the angle in radians

    #print("cross_product calculated:", cross_product)
    #print("dot_product calculated:", dot_product)
    #print("angle calculated (in radians):", angle)

    # Ensure the angle is between -2π and 2π
    if angle > 2 * np.pi:
        angle -= 2 * np.pi
    elif angle < -2 * np.pi:
        angle += 2 * np.pi
    #print("GOD HELP US:", angle)
    return angle


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