import pygame
from dto.shapes import SquareObject, CircleObject, Position
from dto.obstacles import Cross, Wall, WallPlacement
from dto.robot import Robot, Checkpoint, CheckpointType
import image_recognizition.object_detection

screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
dt = 0

left_wall_square = SquareObject.create_square(position=Position(x=224, y=337.25),
                    width=635, height=22, radians=4.71553857)
right_wall_square = SquareObject.create_square(position=Position(x=1077.5, y=346.25),
                    width=655, height=24, radians=4.7070475)
top_wall_square = SquareObject.create_square(position=Position(x=651.5, y=34.25),
                    width=881, height=21, radians=0.00794)
bot_wall_square = SquareObject.create_square(position=Position(x=650, y=658.75),
                    width=873, height=19, radians=3.129)
left_wall = Wall.create(left_wall_square, WallPlacement.LEFT, 100)
right_wall = Wall.create(right_wall_square, WallPlacement.RIGHT, 100)
top_wall = Wall.create(top_wall_square, WallPlacement.TOP, 100)
bot_wall = Wall.create(bot_wall_square, WallPlacement.BOT, 100)


walls = [left_wall, right_wall, top_wall, bot_wall]

cross = Cross.create_cross_with_safe_zones(
    SquareObject.create_square(position=Position(x=-200, y=-200), 
                    width=45, height=5, radians=0.785398),
    SquareObject.create_square(position=Position(x=-200, y=-200), 
                    width=45, height=5, radians=2.35619),
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

ball_radius = 10#4*5/2

balls = [
         #CircleObject(radius=5, position=Position(x=300, y=300)),
         #CircleObject(radius=5, position=Position(x=250, y=240)),
         #CircleObject(radius=5, position=Position(x=250, y=90))
         CircleObject(radius=5, position=Position(x=350, y=200)),
         CircleObject(radius=5, position=Position(x=620, y=360))]
         # CircleObject(radius=5, position=Position(x=750, y=300))]

"""balls = [CircleObject(radius=5, position=Position(x=124, y=534)),
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
robot = Robot.create_robot(position=Position(x=screen.get_width() / 3, y=500),
                width=40, height=40, radians=3.14, suction_height=15, suction_width=15, suction_offset_y=35,
                           previous_checkpoint=Checkpoint(x=screen.get_width() / 3, y=500, checkpoint_type=CheckpointType.SAFE_CHECKPOINT))