import simulation
from src.transmission import send_command
from path_correction import move_towards_checkpoint, shortest_distance_to_line_with_direction
import pygame


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
    player = simulation.player
    
    running = True
    while running:
        game = simulation.init()

    pygame.quit()

