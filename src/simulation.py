import pygame
from pygame import Rect
from dto import Position, Input, RewardValues
from time import sleep
from typing import List
from path_correction import move_towards_checkpoint, shortest_distance_to_line_with_direction
import __init__

screen = __init__.screen
player = __init__.player
obstacles = __init__.obstacles
balls = __init__.balls
clock = __init__.clock

def game(inputs: List[Input]):
    d_forward = 0
    d_radians = 0
    suck = False
    for input in inputs:
        if input == Input.FORWARD:
            d_forward += Input.FORWARD.value
            player.rewards.points_for_moving_forward = player.rewards.points_for_moving_forward + RewardValues.MOVING_FORWARD.value
        elif input == Input.BACKWARD:
            d_forward -= Input.BACKWARD.value
            player.rewards.points_for_moving_forward = player.rewards.points_for_moving_forward + RewardValues.MOVING_FORWARD.value
        elif input == Input.LEFT:
            d_radians += Input.LEFT.value
            player.rewards.points_for_moving_sideways = player.rewards.points_for_moving_sideways + RewardValues.MOVING_SIDEWAYS.value
        elif input == Input.RIGHT:
            d_radians -= Input.RIGHT.value
            player.rewards.points_for_moving_sideways = player.rewards.points_for_moving_sideways + RewardValues.MOVING_SIDEWAYS.value
        elif input == Input.SUCK:
            suck = True
            player.rewards.points_for_suck = player.rewards.points_for_suck + RewardValues.SUCK.value
            
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False

    screen.fill("purple")

    for obstacle in obstacles:
        pygame.draw.polygon(screen, "red", obstacle.vertices)

    for ball in balls:
        pygame.draw.circle(screen, "green", (ball.position.x, ball.position.y), ball.radius)
    
    pygame.draw.polygon(screen, "green", player.player.vertices)
    pygame.draw.polygon(screen, "yellow", player.suction.vertices)
    pygame.draw.line(screen, "white", (player.line.start_pos.x, player.line.start_pos.y), (player.line.end_pos.x, player.line.end_pos.y))

    def create_trail(paths: List[Position], color: str):
        for i, position in enumerate(paths):
            if i % 6 == 0:
                if i % 4 == 0:
                    prev_pos = position

                current_pos_x = position.x
                current_pos_y = position.y
                prev_pos_x = prev_pos.x
                prev_pos_y = prev_pos.y
                pygame.draw.line(screen, color, (prev_pos_x, prev_pos_y), (current_pos_x, current_pos_y))
    
    create_trail(player.previous_path, color="white")


        
    player.move(d_forward, d_radians, obstacles, balls, suck)
    for ball in balls:
        if ball in player.collected_balls:
            balls.remove(ball)

    pygame.display.flip()
    
    # Frames/sec
    clock.tick(60) / 1000
    return player.rewards



def create_inputs(keys):
    """
        This function is in case we want to control the robot manually.
    """
    inputs = []
    if keys[pygame.K_w] or "W" in keys:
        inputs.append(Input.FORWARD)
    if keys[pygame.K_s] or "S" in keys:
        inputs.append(Input.BACKWARD)
    if keys[pygame.K_a] or "A" in keys:
        inputs.append(Input.LEFT)
    if keys[pygame.K_d] or "D" in keys:
        inputs.append(Input.RIGHT)
    if keys[pygame.K_SPACE] or "SPACE" in keys:
        inputs.append(Input.SUCK)
    return inputs



def init():
    inputs = []
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    forwards = move_towards_checkpoint(player)
    direction = shortest_distance_to_line_with_direction(player.line.start_pos, player.line.end_pos, player.checkpoints[0])
    if direction != 0:
        inputs.extend(direction)
    inputs.extend(forwards)
    game(inputs)
    # send_command(inputs)
    

if __name__ == '__main__':

    #init()
    #player = sim.player
    running = True
    while running:
        init()

    pygame.quit()

