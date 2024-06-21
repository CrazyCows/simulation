from typing import List, Tuple
from dto.shapes import CircleObject, SquareObject, Position
from dto.robot import Robot, Checkpoint, CheckpointType, RobotMode
from dto.obstacles import Cross, WallPlacement, Wall
from helper.overlap_detection import square_touching, circle_square_touch
from pydantic import BaseModel
import math


def create_path(balls: List[CircleObject], robot: Robot, walls: List[Wall], cross: Cross) -> Tuple[List[SquareObject], List[Checkpoint]]:
    """
        Creates checkpoints to follow by creating a path between all balls to pick up.

        args:
            List[SquareObject]: Creates a list of square objects mimicking the robots path, including width considerations. 
            List[Checkpoint]: List of checkpoints which the robot must pass.
    """

    temp_ball = sorted(balls, key=lambda ball: robot.calculate_speed_to_ball(ball))[0]
    first_ball = True
    checkpoints = []  # A list of checkpoints the robot must pass
    path: List[SquareObject] = [] # Path is a line with a width to ensure the robot does not touch the obstacles

    def calculate_speed_to_ball(ball_start: CircleObject, ball_end: CircleObject):
        return math.dist((ball_start.position.x, ball_start.position.y), (ball_end.position.x, ball_end.position.y))
    
    temp_path = create_temp_path(robot.robot.position, temp_ball.position)

    p, b = is_ball_close_to_obstacle(temp_ball, walls,cross)
    print(robot.mode, robot.prev_checkpoint.checkpoint_type, len(robot.collected_balls))
    print(robot.mode == RobotMode.DANGER_REVERSE, robot.prev_checkpoint.checkpoint_type == CheckpointType.BALL, len(robot.collected_balls) != 0)
    print(robot.mode == RobotMode.DANGER_REVERSE and robot.prev_checkpoint.checkpoint_type == CheckpointType.BALL and len(robot.collected_balls) != 0)
    if robot.mode == RobotMode.DANGER_REVERSE and robot.prev_checkpoint.checkpoint_type == CheckpointType.BALL and len(robot.collected_balls) != 0:
        p, b = is_ball_close_to_obstacle(robot.collected_balls[-1], walls,cross)
        print(p, robot.robot.position)
        path.append(create_temp_path(robot.robot.position, p))
        path.append(create_temp_path(temp_ball.position, p))
        checkpoints.append(Checkpoint(x=p.x, y=p.y, is_ball=False, checkpoint_type=CheckpointType.SAFE_CHECKPOINT))
        checkpoints.append(Checkpoint(x=temp_ball.position.x, y=temp_ball.position.y, is_ball=True, checkpoint_type=CheckpointType.BALL))
        return path, checkpoints


    if check_if_cross_is_touched(cross, temp_path) or check_if_wall_is_touched(walls, temp_path):
        additional_path, additional_checkpoints = recalculate_path(cross, robot.robot.position, temp_ball.position, robot)
        if len(additional_checkpoints) > 1:
            path.append(create_temp_path(temp_ball.position, additional_checkpoints[1]))
        else:
            path.append(create_temp_path(robot.robot.position, additional_checkpoints[0]))
            path.append(create_temp_path(temp_ball.position, additional_checkpoints[0]))
        path.extend(additional_path)
        checkpoints.extend(additional_checkpoints)
    else:
        if b and (not robot.prev_checkpoint or (robot.prev_checkpoint.x != p.x and robot.prev_checkpoint.y != p.y)):
            path.append(create_temp_path(robot.robot.position, p))
            path.append(create_temp_path(temp_ball.position, p))
            checkpoints.append(Checkpoint(x=p.x, y=p.y, is_ball=False, checkpoint_type=CheckpointType.DANGER_CHECKPOINT))
            checkpoints.append(Checkpoint(x=temp_ball.position.x, y=temp_ball.position.y, is_ball=True, checkpoint_type=CheckpointType.BALL))
        else:
            path.append(temp_path)
            checkpoints.append(Checkpoint(x=temp_ball.position.x, y=temp_ball.position.y, is_ball=True, checkpoint_type=CheckpointType.BALL))
    

    return path, checkpoints


def check_if_cross_is_touched(cross: Cross, current_path: SquareObject):
    if square_touching(cross.square_1, current_path) or square_touching(cross.square_2, current_path):
        return True
    return False

def check_if_wall_is_touched(walls: List[SquareObject], current_path: SquareObject):
    for wall in walls:
        if square_touching(wall, current_path):
            return True
        return False


def recalculate_path(cross: Cross, current_pos: Position, goal_pos: Position, robot: Robot) -> Tuple[List[SquareObject], List[Checkpoint]]:
    # Find the closest safe zones to the current and goal positions
    closest_safezone_to_current_pos = min(cross.safe_zones, key=lambda safe_zone: math.dist((safe_zone.x, safe_zone.y), (current_pos.x, current_pos.y)))
    closest_safezone_to_goal_pos = min(cross.safe_zones, key=lambda safe_zone: math.dist((safe_zone.x, safe_zone.y), (goal_pos.x, goal_pos.y)))

    checkpoints = []
    path = []
    
    # Get indices of the closest safe zones
    start_index = cross.safe_zones.index(closest_safezone_to_current_pos)
    end_index = cross.safe_zones.index(closest_safezone_to_goal_pos)

    if start_index == end_index:
        checkpoints.append(closest_safezone_to_current_pos)
        return [], checkpoints

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
            
            if robot.prev_checkpoint != Checkpoint(x=cross.safe_zones[current_index].x, y=cross.safe_zones[current_index].y, is_ball=False):
                checkpoints.append(Checkpoint(x=cross.safe_zones[current_index].x, y=cross.safe_zones[current_index].y, is_ball=False))
    else:
        # Counterclockwise path
        for i in range(counterclockwise_distance + 1):
            current_index = (start_index - i) % total_zones
            previous_index = (start_index - i + 1) % total_zones
            if i == 0:
                path.append(create_temp_path(current_pos, cross.safe_zones[current_index]))
            else:
                path.append(create_temp_path(cross.safe_zones[previous_index], cross.safe_zones[current_index]))
            
            if robot.prev_checkpoint != Checkpoint(x=cross.safe_zones[current_index].x, y=cross.safe_zones[current_index].y, is_ball=False):
                checkpoints.append(Checkpoint(x=cross.safe_zones[current_index].x, y=cross.safe_zones[current_index].y, is_ball=False))

    # Debugging output
    if check_if_cross_is_touched(cross, path[-1]):
        print("Cross is touched at the last segment of the path.")
    print("Checkpoints:", checkpoints)
    
    return path, checkpoints
        


def create_temp_path(pos_1: Position, pos_2: Position) -> SquareObject:
    # Calculate the difference in x and y coordinates
    dx = pos_1.x - pos_2.x
    dy = pos_1.y - pos_2.y
    angle_radians = math.atan2(dy, dx)

    # Calculate the midpoint position between the first and second ball
    mid_x = (pos_1.x + pos_2.x) / 2
    mid_y = (pos_1.y + pos_2.y) / 2

    # Use this angle and midpoint position in your create_square function call
    temp_path = SquareObject.create_square(
        position=Position(x=mid_x, y=mid_y),
        width=60,
        height=round(math.dist((pos_1.x, pos_1.y), (pos_2.x, pos_2.y))),
        radians=-(angle_radians - math.pi/2)
    )
    return temp_path

def is_ball_close_to_wall(ball: CircleObject, walls: List[Wall]) -> Tuple[Position, bool]:
    x = ball.position.x
    y = ball.position.y
    is_in_danger = False
    for wall in walls:
        if circle_square_touch(ball, wall.danger_zone):
            is_in_danger = True
            if wall.placement == WallPlacement.TOP:
                y += 150
            elif wall.placement == WallPlacement.BOT:
                y -= 150
            elif wall.placement == WallPlacement.LEFT:
                x += 150
            elif wall.placement == WallPlacement.RIGHT:
                x -= 150
    return Position(x=x, y=y), is_in_danger

def is_ball_close_to_cross(ball: CircleObject, cross: Cross) -> Tuple[Position, bool]:
    x = ball.position.x
    y = ball.position.y
    is_in_danger = False
    for wall in cross:
        if circle_square_touch(ball, wall.danger_zone):
            is_in_danger = True
            theta = wall.danger_zone.radians
            perp_x = -math.sin(theta)
            perp_y = math.cos(theta)
            x += perp_x *100
            y += perp_y *100
    return Position(x=x, y=y), is_in_danger


def is_ball_close_to_obstacle(ball: CircleObject, walls: List[Wall], cross: Cross) -> Tuple[Position, bool]:
    dzwc, dzwc_danger = is_ball_close_to_wall(ball, walls)
    if dzwc_danger:
        return dzwc, True
    dzcc, dzcc_danger = is_ball_close_to_cross(ball, cross)
    if dzcc_danger:
        print("DZCC DANGER", dzcc)
        return dzcc, True
    return ball.position, False


