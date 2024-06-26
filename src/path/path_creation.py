from typing import List, Tuple
from dto.shapes import CircleObject, SquareObject, Position
from dto.robot import Robot, Checkpoint, CheckpointType, RobotMode
from dto.obstacles import Cross, WallPlacement, Wall
from helper.overlap_detection import square_touching, circle_square_touch
from pydantic import BaseModel
import math


def create_path(temp_ball: CircleObject, robot: Robot, walls: List[Wall], cross: Cross) -> Tuple[
    List[SquareObject], List[Checkpoint]]:
    """
        Creates checkpoints to follow by creating a path between all balls to pick up.

        args:
            List[SquareObject]: Creates a list of square objects mimicking the robots path, including width considerations.
            List[Checkpoint]: List of checkpoints which the robot must pass.
    """

    # temp_ball = sorted(balls, key=lambda ball: robot.calculate_speed_to_ball(ball))[0]
    first_ball = True
    checkpoints = []  # A list of checkpoints the robot must pass
    path: List[SquareObject] = []  # Path is a line with a width to ensure the robot does not touch the obstacles

    # def calculate_speed_to_ball(ball_start: CircleObject, ball_end: CircleObject):
    #    return math.dist((ball_start.position.x, ball_start.position.y), (ball_end.position.x, ball_end.position.y))
    #print(type(temp_ball))
    #print(type(robot))
    temp_path = create_temp_path(robot.robot.position, temp_ball.position)

    p, b, i = is_ball_close_to_obstacle(temp_ball, walls, cross)
    if b:
        temp_path = create_temp_path(robot.robot.position, p)
    if i > 1:
        robot.ignore_danger_in_corner = True

    # Create Checkpoint to reverse to when robot has picked up a ball that was in danger zone
    """
    if robot.mode == RobotMode.DANGER_REVERSE and robot.prev_checkpoint.checkpoint_type == CheckpointType.BALL and len(
            robot.collected_balls) != 0:
        p, b, i = is_ball_close_to_obstacle(robot.collected_balls[-1], walls, cross)
        print(p, robot.robot.position)
        path.append(create_temp_path(robot.robot.position, p))
        path.append(create_temp_path(temp_ball.position, p))
        checkpoints.append(Checkpoint(x=p.x, y=p.y, checkpoint_type=CheckpointType.DANGER_REVERSE_CHECKPOINT))
        checkpoints.append(Checkpoint(x=temp_ball.position.x, y=temp_ball.position.y,
                                      checkpoint_type=CheckpointType.BALL))
        return path, checkpoints
   """
    # If path collides with obstacles, recalculate route.
    # Else:
    # If ball is in danger zone, create
    next_checkpoint = Checkpoint(x=0, y=0, checkpoint_type=CheckpointType.SAFE_CHECKPOINT)
    if len(robot.checkpoints) > 1:
        next_checkpoint = robot.checkpoints[1]

    if check_if_cross_is_touched(cross, temp_path) or check_if_wall_is_touched(walls, temp_path):
        additional_path, pos = recalculate_path(cross, robot.robot.position, temp_ball.position,
                                                                   robot)
        additional_path, additional_checkpoints = additional_path
        if (robot.mode != RobotMode.DANGER and robot.prev_checkpoint.checkpoint_type != CheckpointType.BALL) or robot.mode == RobotMode.SAFE or robot.mode == RobotMode.DANGER_REVERSE:
            if len(additional_checkpoints) > 1:
                    path.append(create_temp_path(temp_ball.position, additional_checkpoints[1]))
            else:
                    path.insert(0, create_temp_path(robot.robot.position, additional_checkpoints[0]))
                    path.append(create_temp_path(temp_ball.position, additional_checkpoints[0]))
            path.extend(additional_path)
            checkpoints.extend(additional_checkpoints)
            if b:
                if robot.mode != RobotMode.ENDPHASE:
                    checkpoints.append(
                        Checkpoint(x=pos.x, y=pos.y, checkpoint_type=CheckpointType.DANGER_CHECKPOINT))
                    checkpoints.pop(-2)
                    print("ABCD")
        elif robot.mode != RobotMode.ENDPHASE:
            if Checkpoint(x=temp_ball.position.x, y=temp_ball.position.y,
                                          checkpoint_type=CheckpointType.BALL) not in checkpoints:
                checkpoints.append(Checkpoint(x=temp_ball.position.x, y=temp_ball.position.y,
                                          checkpoint_type=CheckpointType.BALL))

    else:
        if b and (not robot.prev_checkpoint or (robot.prev_checkpoint.x != p.x and robot.prev_checkpoint.y != p.y)):
            path.append(create_temp_path(robot.robot.position, p))
            path.append(create_temp_path(temp_ball.position, p))
            if robot.mode != RobotMode.ENDPHASE:
                checkpoints.append(
                    Checkpoint(x=p.x, y=p.y, checkpoint_type=CheckpointType.DANGER_CHECKPOINT))
                checkpoints.append(Checkpoint(x=temp_ball.position.x, y=temp_ball.position.y,
                                          checkpoint_type=CheckpointType.BALL))
            elif robot.prev_checkpoint.checkpoint_type != CheckpointType.GOAL_LINEUP:
                checkpoints.append(
                    Checkpoint(x=p.x, y=p.y, checkpoint_type=CheckpointType.GOAL_LINEUP))
                checkpoints.append(Checkpoint(x=temp_ball.position.x, y=temp_ball.position.y,
                                          checkpoint_type=CheckpointType.GOAL))
            else:
                checkpoints.append(Checkpoint(x=temp_ball.position.x, y=temp_ball.position.y,
                                          checkpoint_type=CheckpointType.GOAL))
        elif robot.mode != RobotMode.ENDPHASE:
            path.append(temp_path)
            checkpoints.append(Checkpoint(x=temp_ball.position.x, y=temp_ball.position.y,
                                          checkpoint_type=CheckpointType.BALL))
        elif robot.prev_checkpoint.checkpoint_type != CheckpointType.GOAL_LINEUP:
            checkpoints.append(
                Checkpoint(x=p.x, y=p.y, checkpoint_type=CheckpointType.GOAL_LINEUP))
            checkpoints.append(Checkpoint(x=temp_ball.position.x, y=temp_ball.position.y,
                                          checkpoint_type=CheckpointType.GOAL))
        else:
            checkpoints.append(Checkpoint(x=temp_ball.position.x, y=temp_ball.position.y,
                                          checkpoint_type=CheckpointType.GOAL))

    return path, checkpoints


def check_if_cross_is_touched(cross: Cross, current_path: SquareObject):
    if square_touching(cross.square_1.danger_zone, current_path) or square_touching(cross.square_2.danger_zone, current_path):
        return True
    return False


def check_if_wall_is_touched(walls: List[SquareObject], current_path: SquareObject):
    for wall in walls:
        if square_touching(wall.danger_zone, current_path):
            return True
        return False


def recalculate_path(cross: Cross, current_pos: Position, goal_pos: Position, robot: Robot):
    # Find the closest safe zones to the current and goal positions
    closest_safezone_to_current_pos = min(cross.safe_zones, key=lambda safe_zone: math.dist((safe_zone.x, safe_zone.y),
                                                                                            (current_pos.x,
                                                                                             current_pos.y)))
    closest_safezone_to_goal_pos = min(cross.safe_zones, key=lambda safe_zone: math.dist((safe_zone.x, safe_zone.y),
                                                                                         (goal_pos.x, goal_pos.y)))

    checkpoints = []
    path = []

    # Get indices of the closest safe zones
    start_index = cross.safe_zones.index(closest_safezone_to_current_pos)
    end_index = cross.safe_zones.index(closest_safezone_to_goal_pos)

    if start_index == end_index:
        checkpoints.append(Checkpoint(
            x=closest_safezone_to_current_pos.x,
            y=closest_safezone_to_current_pos.y,
            checkpoint_type=CheckpointType.SAFE_CHECKPOINT
        ))
        return [[], checkpoints], closest_safezone_to_goal_pos

    total_zones = len(cross.safe_zones)

    # Calculate the distance clockwise and counterclockwise
    clockwise_distance = (end_index - start_index) % total_zones
    counterclockwise_distance = (start_index - end_index) % total_zones

    if clockwise_distance <= counterclockwise_distance:
        # Clockwise path
        for i in range(clockwise_distance + 1):
            current_index = (start_index + i) % total_zones
            previous_index = (start_index + i - 1) % total_zones
            if i == 0:
                path.append(create_temp_path(current_pos, cross.safe_zones[current_index]))
            else:
                path.append(create_temp_path(cross.safe_zones[previous_index], cross.safe_zones[current_index]))

            if robot.prev_checkpoint != Checkpoint(x=cross.safe_zones[current_index].x,
                                                   y=cross.safe_zones[current_index].y):
                checkpoints.append(
                    Checkpoint(x=cross.safe_zones[current_index].x, y=cross.safe_zones[current_index].y,
                               checkpoint_type=CheckpointType.SAFE_CHECKPOINT))
    else:
        # Counterclockwise path
        for i in range(counterclockwise_distance + 1):
            current_index = (start_index - i) % total_zones
            previous_index = (start_index - i + 1) % total_zones
            if i == 0:
                path.append(create_temp_path(current_pos, cross.safe_zones[current_index]))
            else:
                path.append(create_temp_path(cross.safe_zones[previous_index], cross.safe_zones[current_index]))

            if robot.prev_checkpoint != Checkpoint(x=cross.safe_zones[current_index].x,
                                                   y=cross.safe_zones[current_index].y):
                checkpoints.append(
                    Checkpoint(x=cross.safe_zones[current_index].x, y=cross.safe_zones[current_index].y,
                               checkpoint_type=CheckpointType.SAFE_CHECKPOINT)
                )

    # Debugging output
    if check_if_cross_is_touched(cross, path[-1]):
        print("Cross is touched at the last segment of the path.")
    # print("Checkpoints:", checkpoints)

    return [path, checkpoints], closest_safezone_to_goal_pos


def create_temp_path(pos_1: Position, pos_2: Position) -> SquareObject:
    # Calculate the difference in x and y coordinates
    dx = pos_1.x - pos_2.x
    dy = pos_1.y - pos_2.y
    angle_radians = math.atan2(dy, dx)

    # Calculate the midpoint position between the first and second ball
    mid_x = (pos_1.x + pos_2.x) / 2
    mid_y = (pos_1.y + pos_2.y) / 2

    temp_path = SquareObject.create_square(
        position=Position(x=mid_x, y=mid_y),
        width=135,
        height=round(math.dist((pos_1.x, pos_1.y), (pos_2.x, pos_2.y))),
        radians=-(angle_radians - math.pi / 2)
    )
    return temp_path


def is_ball_close_to_wall(ball: CircleObject, walls: List[Wall]) -> Tuple[Position, bool]:
    x = ball.position.x
    y = ball.position.y
    is_in_danger = False
    i = 0
    for wall in walls:
        if circle_square_touch(ball, wall.danger_zone):
            is_in_danger = True
            if wall.placement == WallPlacement.TOP:
                y += 140
                i += 1
            elif wall.placement == WallPlacement.BOT:
                y -= 140
                i += 1
            elif wall.placement == WallPlacement.LEFT:
                x += 140
                i += 1
            elif wall.placement == WallPlacement.RIGHT:
                x -= 140
                i += 1
    return Position(x=x, y=y), is_in_danger, i


def is_ball_close_to_cross(ball: CircleObject, cross: Cross) -> Tuple[Position, bool]:
    x = ball.position.x
    y = ball.position.y
    is_in_danger = False
    i = 0
    walls = [cross.square_1, cross.square_2]
    if circle_square_touch(ball, cross.square_1.danger_zone):
        is_in_danger = True
        theta = cross.square_1.danger_zone.radians
        perp_x = -math.sin(theta)
        perp_y = math.cos(theta)
        x += perp_x * 80
        y += perp_y * 80
    if circle_square_touch(ball, cross.square_2.danger_zone):
        is_in_danger = True
        theta = cross.square_2.danger_zone.radians
        perp_x = -math.sin(theta)
        perp_y = math.cos(theta)
        x += perp_x * 80
        y += perp_y * 80

    return Position(x=x, y=y), is_in_danger, i


def is_ball_close_to_obstacle(ball: CircleObject, walls: List[Wall], cross: Cross) -> Tuple[Position, bool]:
    dzwc, dzwc_danger, dzwc_i = is_ball_close_to_wall(ball, walls)
    if dzwc_danger:
        return dzwc, True, dzwc_i
    dzcc, dzcc_danger, dzcc_i = is_ball_close_to_cross(ball, cross)
    if dzcc_danger:
        return dzcc, True, 0
    return ball.position, False, 0