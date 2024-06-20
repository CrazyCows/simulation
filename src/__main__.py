import gui.visualization as visualization
import transmission
from path import path_creation, path_follow
from dto.robot import Move
from image_recognizition.object_detection import RoboVision
import pygame
import __init__
from typing import List


def app(connect_to_robot: bool = False):
    screen = __init__.screen
    robot = __init__.robot
    balls = __init__.balls
    walls = __init__.walls
    clock = __init__.clock
    cross = __init__.cross
    running = True
    if connect_to_robot:
        transmission.connect

    while running:
        if connect_to_robot:
            print("WHY THE FUCK AM I RUNNING?")
            balls = RoboVision().get_egg()
            robot_position, radians = RoboVision().get_robot()
            robot = robot.create_robot(position=robot_position,
                                       width=30, height=30, radians=radians, suction_height=20, suction_width=20,
                                       suction_offset_y=25)

        # Temp solution, just redrawing balls all da time
        path, checkpoints = path_creation.create_path(balls, robot, walls, cross)
        # checkpoints = [Checkpoint(x=ball.position.x, y=ball.position.y, is_ball=True) for ball in balls]

        robot.checkpoints = checkpoints
        move: Move = path_follow.create_move(robot)
        path_follow.move_robot(move, robot, walls, balls, cross, connect_to_robot)
        if connect_to_robot:
            transmission.send_command(move)

        # NOTE: Updates the visual representation
        visualization.game(screen, robot, walls, balls, path, cross)
        
        # Tickrate, frames/sec.
        clock.tick(60) / 1000

        # Hello
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False



def test_antons_code():
    robo = RoboVision()
    while True:
        thing = robo.get_any_thing(min_count=1, max_count=1, tries=10, thing_to_get="robot")
        if thing is not None:
            break
        print("retrying")

    print(thing)


if __name__ == '__main__':
    test_antons_code()


    #pygame.init()
    #app(False)
    #pygame.quit()
