import math
from typing import List, Tuple
from dto.shapes import CircleObject, SquareObject, Position, WallPosition
from dto.robot import Robot, Checkpoint
from dto.obstacles import Cross
from helper.overlap_detection import square_touching, circle_square_touch
from pydantic import BaseModel

def create_path(
    balls: List[CircleObject],
    robot: Robot,
    walls: List[SquareObject],
    cross: Cross,
    walls_danger_zones: List[SquareObject],
    cross_danger_zones
) -> Tuple[List[SquareObject], List[Checkpoint]]:
    """
    Creates checkpoints to follow by creating a path between all balls to pick up.
    Args:
        List[CircleObject]: List of ball objects to pick up.
        List[SquareObject]: List of wall objects.
        Cross: Cross object with safe zones.
        List[SquareObject]: List of danger zones for walls.
        List[SquareObject]: List of danger zones for cross.
    Returns:
        Tuple: Contains the path as a list of square objects and the checkpoints as a list of Checkpoint objects.
    """
    temp_balls = sorted(balls, key=lambda ball: robot.calculate_speed_to_ball(ball))
    checkpoints = []
    path = []
    first_ball = True

    while temp_balls:
        dzc_position = input_edge_wall(temp_balls[0], walls_danger_zones)
        danger_case = dzc_position != temp_balls[0].position

        if first_ball:
            temp_path = create_temp_path(robot.robot.position, temp_balls[0].position)
            first_ball = False
        else:
            temp_balls.sort(key=lambda ball: calculate_speed_to_ball(ball, temp_balls[0]))
            temp_path = create_temp_path(temp_balls[0].position, temp_balls[1].position)
            temp_balls.pop(0)

        if danger_case:
            checkpoints.append(Checkpoint(x=dzc_position.x, y=dzc_position.y, is_ball=False, danger_point=True))

        if check_if_cross_is_touched(cross, temp_path) or check_if_wall_is_touched(walls, temp_path):
            additional_path, additional_checkpoints = recalculate_path(
                cross, robot.robot.position, temp_balls[0].position, robot, danger_case, dzc_position
            )
            path.extend(additional_path)
            checkpoints.extend(additional_checkpoints)
        else:
            path.append(temp_path)
            checkpoints.append(Checkpoint(
                x=temp_balls[0].position.x,
                y=temp_balls[0].position.y,
                is_ball=True,
                danger_point=False
            ))
        if len(temp_balls) == 1:
            temp_balls = None

    return path, checkpoints

def calculate_speed_to_ball(ball_start: CircleObject, ball_end: CircleObject) -> float:
    return math.dist((ball_start.position.x, ball_start.position.y), (ball_end.position.x, ball_end.position.y))

def check_if_cross_is_touched(cross: Cross, current_path: SquareObject) -> bool:
    return square_touching(cross.square_1, current_path) or square_touching(cross.square_2, current_path)

def check_if_wall_is_touched(walls: List[SquareObject], current_path: SquareObject) -> bool:
    return any(square_touching(wall, current_path) for wall in walls)

def recalculate_path(
    cross: Cross,
    current_pos: Position,
    goal_pos: Position,
    robot: Robot,
    danger_case: bool,
    extra_checkpoint: Position
) -> Tuple[List[SquareObject], List[Checkpoint]]:
    closest_safezone_to_current_pos = sorted(
        cross.safe_zones,
        key=lambda safe_zone: math.dist((safe_zone.x, safe_zone.y), (current_pos.x, current_pos.y))
    )[0]
    closest_safezone_to_goal_pos = sorted(
        cross.safe_zones,
        key=lambda safe_zone: math.dist((safe_zone.x, safe_zone.y), (goal_pos.x, goal_pos.y))
    )[0]

    checkpoints = []
    path = []
    start = cross.safe_zones.index(closest_safezone_to_current_pos)
    end = cross.safe_zones.index(closest_safezone_to_goal_pos)

    if start == end:
        checkpoints.append(cross.safe_zones[start])
        return path, checkpoints

    total_zones = len(cross.safe_zones)
    clockwise_distance = (end - start) % total_zones
    counterclockwise_distance = (start - end) % total_zones

    if clockwise_distance <= counterclockwise_distance:
        calculate_path_segment(cross, start, clockwise_distance, 1, current_pos, robot, path, checkpoints)
    else:
        calculate_path_segment(cross, start, counterclockwise_distance, -1, current_pos, robot, path, checkpoints)

    if check_if_cross_is_touched(cross, path[-1]):
        print("OWO")

    return path, checkpoints

def calculate_path_segment(
    cross: Cross,
    start: int,
    distance: int,
    direction: int,
    current_pos: Position,
    robot: Robot,
    path: List[SquareObject],
    checkpoints: List[Checkpoint]
):
    total_zones = len(cross.safe_zones)
    for i in range(distance + 1):
        current_index = (start + i * direction) % total_zones
        previous_index = (start + (i - 1) * direction) % total_zones
        if i != 0:
            path.append(create_temp_path(cross.safe_zones[previous_index], cross.safe_zones[current_index]))
        else:
            path.append(create_temp_path(current_pos, cross.safe_zones[current_index]))

        current_checkpoint = Checkpoint(
            x=cross.safe_zones[current_index].x,
            y=cross.safe_zones[current_index].y,
            is_ball=False,
            danger_point=False
        )
        if robot.prev_checkpoint == current_checkpoint:
            path.pop(0)
            continue
        checkpoints.append(current_checkpoint)

def create_temp_path(pos_1: Position, pos_2: Position) -> SquareObject:
    dx = pos_1.x - pos_2.x
    dy = pos_1.y - pos_2.y
    angle_radians = math.atan2(dy, dx)
    mid_x = (pos_1.x + pos_2.x) / 2
    mid_y = (pos_1.y + pos_2.y) / 2

    return SquareObject.create_square(
        position=Position(x=mid_x, y=mid_y),
        width=60,
        height=round(math.dist((pos_1.x, pos_1.y), (pos_2.x, pos_2.y))),
        radians=-(angle_radians - math.pi / 2)
    )

def input_edge_wall(ball: CircleObject, walls_danger_zones: List[SquareObject]) -> Position:
    x = ball.position.x
    y = ball.position.y
    for danger_zone in walls_danger_zones:
        if circle_square_touch(ball, danger_zone):
            if danger_zone.wallPosition == WallPosition.TOP:
                y -= 100
            elif danger_zone.wallPosition == WallPosition.BOT:
                y += 100
            elif danger_zone.wallPosition == WallPosition.LEFT:
                x += 200
            elif danger_zone.wallPosition == WallPosition.RIGHT:
                x -= 100
    return Position(x=x, y=y)
