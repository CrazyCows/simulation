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
    balls = __init__.balls
    walls = __init__.walls
    clock = __init__.clock
    cross = __init__.cross
    running = True
    #if connect_to_robot:
    #    transmission.connect
    focused_ball: CircleObject = None
    if (connect_to_robot):
        wp = WallPicker()
        walls = [wp.pick_east_wall(), wp.pick_north_wall(), wp.pick_south_wall(), wp.pick_west_wall()]
        rv = RoboVision()
        cross_squares = wp.click_cross()
        print("Here")
        cross = Cross.create_cross_with_safe_zones(square_1=cross_squares[0], square_2=cross_squares[1], walls=walls, safe_distance=20)
    while running:
        if connect_to_robot:
            print("WHY THE FUCK AM I RUNNING?")
            balls = RoboVision().get_any_thing(min_count=0, max_count=20, tries=100, thing_to_get="white_ball")
            temprobo = RoboVision().get_any_thing(min_count=1, max_count=1, tries=200, thing_to_get="robot")

            robot_position = temprobo.position
            #print(robot_position)
            radians = temprobo.radians
            robot = robot.create_robot(position=Position(x=robot_position.x, y=robot_position.y),
                                        width=30, height=30, radians=radians, suction_height=20, suction_width=20,
                                        suction_offset_y=25)
        tmp_walls = []
        tmp_walls.append(Wall.create(walls[0], WallPlacement.LEFT))
        tmp_walls.append(Wall.create(walls[1], WallPlacement.RIGHT))
        tmp_walls.append(Wall.create(walls[2], WallPlacement.TOP))
        tmp_walls.append(Wall.create(walls[3], WallPlacement.BOT))
        def calculate_speed_to_ball(ball_start: CircleObject, ball_end: CircleObject):
            return dist((ball_start.position.x, ball_start.position.y), (ball_end.position.x, ball_end.position.y))
        walls = tmp_walls
        if robot.mode != RobotMode.DANGER:
            focused_ball = sorted(balls, key=lambda ball: robot.calculate_speed_to_ball(ball))[0]

        # Temp solution, just redrawing balls all da time
        if len(balls) > 0:
            
            path, checkpoints = path_creation.create_path(focused_ball, robot, walls, cross)
            robot.checkpoints = checkpoints
            try:
                move: Move = path_follow.create_move(robot)
                path_follow.move_robot(move, robot, walls, balls, cross, connect_to_robot)
            except Exception as e:
                continue
            #print("Left: ", robot.distance_to_wall_left)
            print("Right: ", robot.distance_to_wall_right)
            #print("Top: ", robot.distance_to_wall_top)
            #print("Bot: ", robot.distance_to_wall_bot)
            #print("Cross: ", robot.distance_to_cross)
            #if connect_to_robot:
            #    transmission.send_command(move)

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
        thing = robo.get_any_thing(min_count=1, max_count=1, tries=10, thing_to_get="robot")
        if thing is not None:
            pass
        print("retrying")

    print(thing)


if __name__ == '__main__':
    #test_antons_code()
    pygame.init()
    app(False)
    pygame.quit()
