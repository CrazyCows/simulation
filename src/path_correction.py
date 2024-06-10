from dto import Player, Input, Position
import math
import numpy as np


def shortest_distance_to_line_with_direction(start_pos: Position, end_pos: Position, ball_pos: Position):
    # Convert the inputs to numpy arrays for easier manipulation
    start_pos = (start_pos.x, start_pos.y)
    end_pos = (end_pos.x, end_pos.y)
    coordinate = (ball_pos.x, ball_pos.y)
    start = np.array(start_pos)
    end = np.array(end_pos)
    point = np.array(coordinate)
    
    # Calculate the line vector and the point vector
    line_vector = end - start
    point_vector = point - start
    
    # Calculate the projection of the point vector onto the line vector
    line_length_squared = np.dot(line_vector, line_vector)
    if line_length_squared == 0:
        return np.linalg.norm(point_vector), "indeterminate"  # The start and end positions are the same
    
    t = max(0, min(1, np.dot(point_vector, line_vector) / line_length_squared))
    
    # Calculate the nearest point on the line segment to the point
    nearest_point = start + t * line_vector
    
    # Calculate the distance from the point to the nearest point on the line segment
    distance = np.linalg.norm(point - nearest_point)
    if distance < 5:
        return 0
    # Determine the direction to turn (left or right)
    # We can use the cross product of the line vector and point vector to determine the direction
    cross_product = np.cross(line_vector, point_vector)
    direction = []

    if cross_product < 0:
        direction.append(Input.LEFT)
        for i in range (round(distance / 100)):
            direction.append(Input.LEFT)
    elif cross_product > 0:
        direction.append(Input.RIGHT)
        for i in range (round(distance / 100)):
            direction.append(Input.RIGHT)
    else:
        direction = 0  # This happens when the point is directly on the line
    return direction

def move_towards_checkpoint(player: Player):
    inputs = []
    checkpoint_pos = (player.checkpoints[0].x, player.checkpoints[0].y)
    current_player_balls = len(player.collected_balls)
    player_pos = (player.player.position.x, player.player.position.y)
    distance_to_checkpoint = math.dist(player_pos, checkpoint_pos)
    distance_to_ball = 0
    for checkpoint in player.checkpoints:
        if checkpoint.is_ball:
            distance_to_ball = math.dist(player_pos, (checkpoint.x, checkpoint.y))
            break
    if distance_to_ball < 25:
        inputs.append(Input.SUCK)
    if distance_to_checkpoint < 15:
        player.checkpoints.pop(0)

    inputs.append(Input.FORWARD)
    return inputs