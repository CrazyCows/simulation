import pygame
from dto.shapes import SquareObject, CircleObject, Position, WallPosition
from dto.obstacles import Cross
from dto.robot import Robot
import image_recognizition.object_detection

screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
dt = 0

left_wall = SquareObject.create_square(position=Position(x=40, y=screen.get_height() / 2), 
                    width=20, height=700, radians=0, wallPosition=WallPosition.LEFT)
right_wall = SquareObject.create_square(position=Position(x=screen.get_width() - 40, y=screen.get_height() / 2), 
                    width=20, height=700, radians=0, wallPosition=WallPosition.RIGHT)
top_wall = SquareObject.create_square(position=Position(x=screen.get_width() - screen.get_width() / 2, y=20), 
                    width=1200, height=20, radians=0, wallPosition=WallPosition.TOP)
bot_wall = SquareObject.create_square(position=Position(x=screen.get_width() - screen.get_width() / 2, y=screen.get_height() - 20), 
                    width=1200, height=20, radians=0, wallPosition=WallPosition.BOT)

walls = [left_wall, right_wall, top_wall, bot_wall]

cross = Cross.create_cross_with_safe_zones(
    SquareObject.create_square(position=Position(x=screen.get_width() - screen.get_width() / 2, y=screen.get_height() / 2), 
                    width=220, height=30, radians=0.785398), 
    SquareObject.create_square(position=Position(x=screen.get_width() - screen.get_width() / 2, y=screen.get_height() / 2), 
                    width=220, height=30, radians=2.35619),
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
walls_danger_zones = []
for wall in walls:
    danger_zone = SquareObject.create_square(position=wall.position, width=wall.width+40, height=wall.height+40, radians=wall.radians, danger_zone=True, wallPosition= wall.wallPosition)
    walls_danger_zones.append(danger_zone)

danger_zone_cross_1 = SquareObject.create_square(position=cross.square_1.position, width=cross.square_1.width+40, height=cross.square_1.height+40, radians=cross.square_1.radians, danger_zone=True)
danger_zone_cross_2 = SquareObject.create_square(position=cross.square_2.position, width=cross.square_2.width+40, height=cross.square_2.height+40, radians=cross.square_2.radians, danger_zone=True)
cross_danger_zones = [danger_zone_cross_1, danger_zone_cross_2]



balls = [
         CircleObject(radius=10, position=Position(x=60, y=screen.get_height() / 2)),
]

"""
balls = [CircleObject(radius=5, position=Position(x=124, y=534)),
         CircleObject(radius=5, position=Position(x=872, y=231)),
         CircleObject(radius=5, position=Position(x=512, y=333)),
         CircleObject(radius=5, position=Position(x=432, y=90)),
         CircleObject(radius=5, position=Position(x=144, y=666)),
         CircleObject(radius=5, position=Position(x=963, y=155)),
         CircleObject(radius=5, position=Position(x=1111, y=514)),
         CircleObject(radius=5, position=Position(x=893, y=625)),
         CircleObject(radius=5, position=Position(x=231, y=403)),]
         #CircleObject(radius=5, position=Position(x=robot.suction.position.x + robot.suction.offset_x, y=robot.suction.position.y + robot.suction.offset_y)),]
"""
robot = Robot.create_robot(position=Position(x=screen.get_width() / 2, y=625), 
                width=60, height=30, radians=1.5, suction_height=20, suction_width=20, suction_offset_y=25)
