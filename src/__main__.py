import cv2
import numpy as np

import gui.visualization as visualization
import transmission
from path import path_creation, path_follow
from dto.robot import Move, Checkpoint
from dto.shapes import Position
from dto.obstacles import Cross
from image_recognizition.object_detection import RoboVision
from image_recognizition.wall_picker import WallPicker
import pygame
import __init__

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

    if connect_to_robot:
        wp = WallPicker()
        walls = [wp.pick_east_wall(), wp.pick_north_wall(), wp.pick_south_wall(), wp.pick_west_wall()]
        rv = RoboVision()
        cross_squares = wp.pick_cross()
        print("Here")
        cross = Cross.create_cross_with_safe_zones(square_1=cross_squares[0], square_2=cross_squares[1], walls=walls, safe_distance=20)
    while running:
        try:
            if connect_to_robot:
                #print("WHY THE FUCK AM I RUNNING?")
                balls = rv.get_any_thing(min_count=0, max_count=20, tries=100, thing_to_get="white_ball")
                temprobo = rv.get_any_thing(min_count=1, max_count=1, tries=200, thing_to_get="robot")

                robot_position = temprobo.position
                #print(robot_position)
                radians = temprobo.radians
                robot = robot.create_robot(position=Position(x=robot_position.x, y=robot_position.y),
                                           width=30, height=30, radians=radians, suction_height=20, suction_width=20,
                                           suction_offset_y=25)

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


def test_antons_code():
    robo = RoboVision()
    while True:
        thing = robo.get_any_thing(min_count=1, max_count=1, tries=10, thing_to_get="robot")
        if thing is not None:
            pass
        print("retrying")

    print(thing)


if __name__ == '__main__':
    #test_antons_code()
    pygame.init()
    app(True)
    pygame.quit()
