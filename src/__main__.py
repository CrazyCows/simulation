import cv2
import numpy as np

import gui.visualization as visualization
import transmission
from path import path_creation, path_follow
from dto.robot import Move, Checkpoint, RobotMode
from dto.shapes import Position, CircleObject
from dto.obstacles import Cross, Wall, WallPlacement
from image_recognizition.object_detection import RoboVision
from image_recognizition.wall_picker import WallPicker
import pygame
import __init__
from math import dist

from typing import List
from image_recognizition import wall_picker


def app(connect_to_robot: bool = False):
    screen = __init__.screen
    robot = __init__.robot
    #balls = __init__.balls
    #walls = __init__.walls
    clock = __init__.clock
    #cross = __init__.cross
    running = True
    #if connect_to_robot:
    #    transmission.connect
    focused_ball: CircleObject = None
    if connect_to_robot:
        wp = WallPicker()
        walls = [wp.pick_east_wall(), wp.pick_north_wall(), wp.pick_west_wall(), wp.pick_south_wall()]
        rv = RoboVision(walls)
        cross_squares = wp.pick_cross()
        cross = Cross.create_cross_with_safe_zones(square_1=cross_squares[0], square_2=cross_squares[1], walls=walls, safe_distance=20)
    while running:
        try:
            if connect_to_robot:
                #print("WHY THE FUCK AM I RUNNING?")
                balls = rv.get_any_thing(min_count=0, max_count=20, tries=100, thing_to_get="white_ball")
                temp_robo = rv.get_any_thing(min_count=1, max_count=1, tries=200, thing_to_get="robot")

                robot_position = temp_robo.position
                #print(robot_position)
                radians = temp_robo.radians
                robot = robot.create_robot(position=Position(x=robot_position.x, y=robot_position.y),
                                           width=30, height=30, radians=radians, suction_height=20, suction_width=20,
                                           suction_offset_y=25)
            robot_position = temp_robo.position
            #print(robot_position)
            radians = temp_robo.radians
            robot = robot.create_robot(position=Position(x=robot_position.x, y=robot_position.y),
                                        width=30, height=30, radians=radians, suction_height=20, suction_width=20,
                                        suction_offset_y=25)
        """tmp_walls = []
        tmp_walls.append(Wall.create(walls[0], WallPlacement.LEFT))
        tmp_walls.append(Wall.create(walls[1], WallPlacement.RIGHT))
        tmp_walls.append(Wall.create(walls[2], WallPlacement.TOP))
        tmp_walls.append(Wall.create(walls[3], WallPlacement.BOT))
        def calculate_speed_to_ball(ball_start: CircleObject, ball_end: CircleObject):
            return dist((ball_start.position.x, ball_start.position.y), (ball_end.position.x, ball_end.position.y))
        walls = tmp_walls
        """
        if robot.mode != RobotMode.DANGER:
            focused_ball = sorted(balls, key=lambda ball: robot.calculate_speed_to_ball(ball))[0]

            # Temp solution, just redrawing balls all da time
            path, checkpoints = path_creation.create_path(balls, robot, walls, cross)
            checkpoints = [Checkpoint(x=ball.position.x, y=ball.position.y, is_ball=True) for ball in balls]

            robot.checkpoints = checkpoints
            try:
                move: Move = path_follow.create_move(robot)
                path_follow.move_robot(move, robot, walls, balls, cross, connect_to_robot)
            except Exception as e:
                continue
            #if connect_to_robot:
            #    transmission.send_command(move)

            # NOTE: Updates the visual representation
            cam_frame = rv.get_flipped_frame()
            cam_frame = cv2.cvtColor(cam_frame, cv2.COLOR_BGR2RGB)
            frame = pygame.surfarray.make_surface(np.rot90(cam_frame))
            screen.blit(frame, (0, 0))
            #pygame.display.flip()
            visualization.game(screen, robot, walls, balls, path, cross)

        # Tickrate, frames/sec.
            clock.tick(60) / 1000

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
        except Exception as e:
            raise e


if __name__ == '__main__':
    #test_antons_code()
    pygame.init()
    app(True)
    pygame.quit()
