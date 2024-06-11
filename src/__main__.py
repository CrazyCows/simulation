import simulation
import transmission
from dto import Inputs, Player
import path_follow
import pygame
import __init__
from typing import List

screen = __init__.screen
player = __init__.player
balls = __init__.balls
obstacles = __init__.obstacles
clock = __init__.clock


def app(connect_to_robot: bool = False):
    running = True
    if connect_to_robot:
        transmission.connect

    while running:
        inputs: Inputs = path_follow.create_inputs(player)
        path_follow.move_player(inputs, player, obstacles, balls)

        if connect_to_robot:
            transmission.send_command(inputs)

        # NOTE: Updates the visual representation
        simulation.game(screen, player, obstacles, balls)
        
        # Tickrate, frames/sec.
        clock.tick(60) / 1000

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

if __name__ == '__main__':
    app()
    pygame.quit()

