import pygame
from dto.shapes import SquareObject, CircleObject, Position
from dto.obstacles import Cross
from dto.robot import Robot
import image_recognizition.object_detection

screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
dt = 0

left_wall = SquareObject.create_square(position=Position(x=230, y=screen.get_height() / 2),
                    width=20, height=590, radians=0)
right_wall = SquareObject.create_square(position=Position(x=1050, y=screen.get_height() / 2),
                    width=20, height=590, radians=0)
top_wall = SquareObject.create_square(position=Position(x=screen.get_width() - screen.get_width() / 2, y=65),
                    width=840, height=20, radians=0)
bot_wall = SquareObject.create_square(position=Position(x=screen.get_width() - screen.get_width() / 2, y=655),
                    width=840, height=20, radians=0)

walls = [left_wall, right_wall, top_wall, bot_wall]

cross = Cross.create_cross_with_safe_zones(
    SquareObject.create_square(position=Position(x=screen.get_width() - screen.get_width() / 2, y=screen.get_height() / 2), 
                    width=60, height=15, radians=0.785398),
    SquareObject.create_square(position=Position(x=screen.get_width() - screen.get_width() / 2, y=screen.get_height() / 2), 
                    width=60, height=15, radians=2.35619),
                    walls,
                    150            
        )
"""
cross = Cross.create_cross_with_safe_zones(
    SquareObject.create_square(position=Position(x=-200, y=-200),
                    width=220, height=30, radians=0.785398),
    SquareObject.create_square(position=Position(x=-200, y=-200),
                    width=220, height=30, radians=2.35619),
                    walls,
                    150
        )
"""

ball_radius = 4*5/2

balls = [CircleObject(radius=ball_radius, position=Position(x=250, y=screen.get_height() / 2)),
         CircleObject(radius=ball_radius, position=Position(x=1030, y=514)),
         CircleObject(radius=ball_radius, position=Position(x=253, y=88)),
         #CircleObject(radius=ball_radius, position=Position(x=350, y=160))
         #CircleObject(radius=ball_radius, position=Position(x=screen.get_width() - screen.get_width() / 2-30, y=screen.get_height() / 2)),
         ]
"""
balls = [CircleObject(radius=5, position=Position(x=screen.get_width() - screen.get_width() / 2-60, y=screen.get_height() / 2)),
         CircleObject(radius=5, position=Position(x=872, y=231)),
         CircleObject(radius=5, position=Position(x=512, y=333)),
         CircleObject(radius=5, position=Position(x=432, y=90)),
         CircleObject(radius=5, position=Position(x=944, y=666)),
         CircleObject(radius=5, position=Position(x=144, y=666)),
         CircleObject(radius=5, position=Position(x=963, y=155)),
         CircleObject(radius=5, position=Position(x=1111, y=514)),
         CircleObject(radius=5, position=Position(x=893, y=225)),
         CircleObject(radius=5, position=Position(x=231, y=403)),]
         #CircleObject(radius=5, position=Position(x=robot.suction.position.x + robot.suction.offset_x, y=robot.suction.position.y + robot.suction.offset_y)),]
"""
robot = Robot.create_robot(position=Position(x=screen.get_width() / 3, y=550),
                width=135, height=150, radians=3.14, suction_height=25, suction_width=5, suction_offset_y=83)