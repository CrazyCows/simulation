import pygame
from pygame import Rect
from dto import Player, Position, SquareObject, Wall

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
dt = 0

player = Player.create_player(position=Position(x=screen.get_width() / 2, y=screen.get_width() / 2), width=10, height=10, degrees=90, suction_height=10, suction_width=10)
left_wall = Wall(position=Position(), screen.get_width() * 0.01, screen.get_height() * 0.9)


while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # fill the screen with a color to wipe away anything from last frame
    screen.fill("purple")
    top_wall = [screen.get_width() * 0.025, 20, screen.get_width() * 0.95, screen.get_height() * 0.05]
    bottom_wall = [screen.get_width() * 0.025, screen.get_height() - 20 - screen.get_height() * 0.05, screen.get_width() * 0.95, screen.get_height() * 0.05]
    left_wall = [screen.get_width() * 0.025, screen.get_height() * 0.05, screen.get_height() * 0.05, screen.get_height() * 0.9]
    right_wall = [screen.get_width() - screen.get_width() * 0.025 - screen.get_height() * 0.05, screen.get_height() * 0.05, screen.get_height() * 0.05, screen.get_height() * 0.9]

    pygame.draw.rect(screen, "red", Rect(*top_wall)) # Top wall
    pygame.draw.rect(screen, "red", Rect(*bottom_wall)) # Bottom wall
    pygame.draw.rect(screen, "red", Rect(*left_wall)) # Bottom wall
    pygame.draw.rect(screen, "red", Rect(*right_wall)) # Bottom wall

    trect = Rect(0, 0, player.width, player.height)
    pygame.draw.rect(screen, "red", trect)

    # flip() the display to put your work on screen
    pygame.display.flip()

    # limits FPS to 60
    # dt is delta time in seconds since last frame, used for framerate-
    # independent physics.
    dt = clock.tick(60) / 1000

pygame.quit()