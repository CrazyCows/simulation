import pygame
from pygame import Rect
from dto import Player, Position, SquareObject
from time import sleep
# pygame setup
pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
dt = 0

player = Player.create_player(position=Position(x=screen.get_width() / 2, y=screen.get_width() / 2), 
                width=15, height=15, radians=0, suction_height=10, suction_width=10, suction_offset_y=15)
# left_wall = Wall(position=Position(), screen.get_width() * 0.01, screen.get_height() * 0.9)
left_wall = SquareObject.create_square(position=Position(x=40, y=screen.get_height() / 2), 
                    width=20, height=700, radians=0)
right_wall = SquareObject.create_square(position=Position(x=screen.get_width() - 40, y=screen.get_height() / 2), 
                    width=20, height=700, radians=0)
top_wall = SquareObject.create_square(position=Position(x=screen.get_width() - screen.get_width() / 2, y=20), 
                    width=1200, height=20, radians=0)
bot_wall = SquareObject.create_square(position=Position(x=screen.get_width() - screen.get_width() / 2, y=screen.get_height() - 20), 
                    width=1200, height=20, radians=0)

obstacles = [left_wall, right_wall, top_wall, bot_wall]


pygame.Surface((50,50), pygame.SRCALPHA)

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False


    # fill the screen with a color to wipe away anything from last frame
    screen.fill("purple")

    for obstacle in obstacles:
        pygame.draw.polygon(screen, "red", obstacle.vertices)
    pygame.draw.polygon(screen, "green", player.player.vertices)
    pygame.draw.polygon(screen, "yellow", player.suction.vertices)
    #print("hi")
    print(player.player.position.y)
    sleep(0.1)
    player.move(-1,0.025, obstacles)

    # flip() the display to put your work on screen
    pygame.display.flip()

    # limits FPS to 60
    # dt is delta time in seconds since last frame, used for framerate-
    # independent physics.
    dt = clock.tick(60) / 1000

pygame.quit()