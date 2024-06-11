import pygame
from dto import Position, SquareObject, CircleObject, Checkpoint, Player

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

player = Player.create_player(position=Position(x=screen.get_width() / 2, y=625), 
                width=30, height=30, radians=0, suction_height=20, suction_width=20, suction_offset_y=25)
