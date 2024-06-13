import gui.visualization as visualization
import transmission
import path_creation
from dto.robot import Move
from object_detection import RoboVision
import path_follow
import pygame
import __init__
from typing import List


def app(connect_to_robot: bool = False):
    screen = __init__.screen
    robot = __init__.robot
    balls = __init__.balls
    obstacles = __init__.obstacles
    clock = __init__.clock
    running = True
    if connect_to_robot:
        transmission.connect

    while running:
        if connect_to_robot:
            print("WHY THE FUCK AM I RUNNING?")
            balls = RoboVision().get_egg()
            robot_position, radians = RoboVision().get_robot()
            robot = robot.create_robot(position=robot_position, 
                            width=30, height=30, radians=radians, suction_height=20, suction_width=20, suction_offset_y=25)
    
        # Temp solution, just redrawing balls all da time
        path, checkpoints = path_creation.create_path(balls, robot, obstacles)
        # checkpoints = [Checkpoint(x=ball.position.x, y=ball.position.y, is_ball=True) for ball in balls]
        robot.checkpoints = checkpoints
        move : Move = path_follow.create_move(robot)
        path_follow.move_robot(move, robot, obstacles, balls, connect_to_robot)
        if connect_to_robot:
            transmission.send_command(move)
    
        # NOTE: Updates the visual representation
        visualization.game(screen, robot, obstacles, balls, path)
        
        # Tickrate, frames/sec.
        clock.tick(60) / 1000

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

if __name__ == '__main__':
    pygame.init()
    app(False)
    pygame.quit()

