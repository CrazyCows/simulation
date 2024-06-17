from typing import List, Tuple
from dto.shapes import CircleObject, SquareObject, Position
from dto.robot import Robot, Checkpoint
from dto.obstacles import Cross
from helper.overlap_detection import square_touching
from pydantic import BaseModel
import math

# TODO: Implement moving around obstacles
def create_path(balls: List[CircleObject], robot: Robot, walls: List[SquareObject], cross: Cross) -> Tuple[List[SquareObject], List[Checkpoint]]:
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
    prev_ball = temp_balls[0]
    while temp_balls:
        if first_ball:
            temp_path = create_temp_path(robot.robot.position, temp_balls[0].position)
            first_ball = False
        else:
            sort_ball = temp_balls[0]
            temp_balls.sort(key=lambda ball: calculate_speed_to_ball(ball, sort_ball))
            temp_path = create_temp_path(temp_balls[0].position, temp_balls[1].position)
            prev_ball = temp_balls[0]
            temp_balls.pop(0)

        if check_if_cross_is_touched(cross, temp_path) or check_if_wall_is_touched(walls, temp_path):
            additional_path, additional_checkpoints = recalculate_path(cross, prev_ball.position, temp_balls[0].position)
            if len(additional_checkpoints) > 1:
                path.append(create_temp_path(temp_balls[0].position, additional_checkpoints[1]))
            else:
                path.append(create_temp_path(robot.robot.position, additional_checkpoints[0]))
                path.append(create_temp_path(temp_balls[0].position, additional_checkpoints[0]))
            path.extend(additional_path)
            checkpoints.extend(additional_checkpoints)
            #path.append(create_temp_path(prev_ball.position, additional_checkpoints[1]))
        else:
            path.append(temp_path)  
            checkpoints.append(Checkpoint(x=temp_balls[0].position.x, y=temp_balls[0].position.y, is_ball=True))

        #for wall in walls: 
        #    if obstacles_hit:
        #        break
        #    obstacles_hit = square_touching(wa)

        if len(temp_balls) == 1:
            temp_balls = None
        
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


def recalculate_path(cross: Cross, current_pos: Position, goal_pos: Position) -> Tuple[List[SquareObject], List[Checkpoint]]:
    closest_safezone_to_current_pos = sorted(cross.safe_zones, key=lambda safe_zone: math.dist((safe_zone.x, safe_zone.y), (current_pos.x, current_pos.y)))
    closest_safezone_to_goal_pos = sorted(cross.safe_zones, key=lambda safe_zone: math.dist((safe_zone.x, safe_zone.y), (goal_pos.x, goal_pos.y)))
    checkpoints = []
    path = []
    start = None
    end = None
    
    for i, zone in enumerate(cross.safe_zones):
        if closest_safezone_to_current_pos[0] == zone:
            start = i
        if closest_safezone_to_goal_pos[0] == zone:
            end = i

    if start == end:
        checkpoints.append(cross.safe_zones[start])
        return [], checkpoints

    total_zones = len(cross.safe_zones)
    
    # Calculate the distance clockwise and counterclockwise
    clockwise_distance = (end - start) % total_zones
    counterclockwise_distance = (start - end) % total_zones
    last_i = 0
    if clockwise_distance <= counterclockwise_distance:
        # If the clockwise distance is shorter or equal
        for i in range(clockwise_distance + 1):
            # checkpoint_start = cross.safe_zones.c
            current_index = (start + i) % total_zones
            previous_index = (start + i - 1) % total_zones
            if i != 0:
                path.append(create_temp_path(cross.safe_zones[previous_index], cross.safe_zones[current_index]))
            else:
                path.append(create_temp_path(current_pos, cross.safe_zones[current_index]))

            checkpoints.append(Checkpoint(x=cross.safe_zones[current_index].x, y=cross.safe_zones[current_index].y, is_ball=False))
            last_i = i

        #if check_if_cross_is_touched(cross, path):
        #    checkpoints.append(Checkpoint(x=cross.safe_zones[current_index].x, y=cross.safe_zones[current_index].y, is_ball=False))

    else:
        # If the counterclockwise distance is shorter
        for i in range(counterclockwise_distance + 1):
        #while check_if_cross_is_touched(cross, path):
            current_index = (start - i) % total_zones
            previous_index = (start - i + 1) % total_zones
            if i != 0:
                path.append(create_temp_path(cross.safe_zones[previous_index], cross.safe_zones[current_index]))
            else:
                path.append(create_temp_path(current_pos, cross.safe_zones[current_index]))
            checkpoints.append(Checkpoint(x=cross.safe_zones[current_index].x, y=cross.safe_zones[current_index].y, is_ball=False))
    if check_if_cross_is_touched(cross, path[len(path) - 1]):
        print("OWO")
            
            
    print("'''''")
    print(checkpoints)
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

def check_for_obstacles():
    ""

