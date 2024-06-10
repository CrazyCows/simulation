import pygame
from pygame import Rect
from dto import Player, Position, SquareObject, CircleObject, Input, RewardValues, Checkpoint
from time import sleep
from enum import Enum
from typing import List
from path_correction import move_towards_checkpoint, shortest_distance_to_line_with_direction
from controller import send_command

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
dt = 0


left_wall = SquareObject.create_square(position=Position(x=40, y=screen.get_height() / 2), 
                    width=20, height=700, radians=0)
right_wall = SquareObject.create_square(position=Position(x=screen.get_width() - 40, y=screen.get_height() / 2), 
                    width=20, height=700, radians=0)
top_wall = SquareObject.create_square(position=Position(x=screen.get_width() - screen.get_width() / 2, y=20), 
                    width=1200, height=20, radians=0)
bot_wall = SquareObject.create_square(position=Position(x=screen.get_width() - screen.get_width() / 2, y=screen.get_height() - 20), 
                    width=1200, height=20, radians=0)

cross_1 = SquareObject.create_square(position=Position(x=screen.get_width() - screen.get_width() / 2, y=screen.get_height() / 2), 
                    width=160, height=20, radians=0.785398)
cross_2 = SquareObject.create_square(position=Position(x=screen.get_width() - screen.get_width() / 2, y=screen.get_height() / 2), 
                    width=160, height=20, radians=2.35619)

obstacles = [left_wall, right_wall, top_wall, bot_wall, cross_1, cross_2]

balls = [CircleObject(radius=5, position=Position(x=124, y=534)),
         CircleObject(radius=5, position=Position(x=872, y=231)),
         CircleObject(radius=5, position=Position(x=512, y=333)),
         CircleObject(radius=5, position=Position(x=432, y=90)),
         CircleObject(radius=5, position=Position(x=144, y=666)),
         CircleObject(radius=5, position=Position(x=963, y=155)),
         CircleObject(radius=5, position=Position(x=1111, y=514)),
         CircleObject(radius=5, position=Position(x=893, y=625)),
         CircleObject(radius=5, position=Position(x=231, y=403)),]
         #CircleObject(radius=5, position=Position(x=player.suction.position.x + player.suction.offset_x, y=player.suction.position.y + player.suction.offset_y)),]

checkpoints = [Checkpoint(x=ball.position.x, y=ball.position.y, is_ball=True) for ball in balls]
player = Player.create_player(position=Position(x=screen.get_width() / 2, y=625), 
                width=30, height=30, radians=0, suction_height=20, suction_width=20, suction_offset_y=25, checkpoints=checkpoints)

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
    return player.rewards


def create_inputs(keys):
    """
        args:
            keys: Takes in a pygame command or a regular string
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

def init_game():
    radians = 0
    d_forward = 0
    suck = False
    inputs = []
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    

    # direction = face_checkpoint(player)
    forwards = move_towards_checkpoint(player)
    direction = shortest_distance_to_line_with_direction(player.line.start_pos, player.line.end_pos, player.checkpoints[0])
    if direction != 0:
        inputs.extend(direction)
    inputs.extend(forwards)
    game(inputs)
    send_command(inputs)

    #change back to 60
    dt = clock.tick(2) / 1000

if __name__ == '__main__':
    
    while running:
        init_game()

    pygame.quit()

