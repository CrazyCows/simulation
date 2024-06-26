import math

import gui.visualization as visualization
import transmission
from path import path_creation, path_follow
from dto.robot import Move, Checkpoint, RobotMode, CheckpointType
from dto.shapes import Position, CircleObject, Goal, SquareObject
from dto.obstacles import Cross, Wall, WallPlacement
from image_recognizition.object_detection import RoboVision
from image_recognizition.wall_picker import WallPicker
import pygame
import __init__
from math import dist
import numpy as np
import cv2
from typing import List, Tuple
from image_recognizition import wall_picker
from time import sleep

def find_focused_ball(focused_ball, balls):
    for ball in balls:
        if euclidean_distance((focused_ball.position.x, focused_ball.position.y), (ball.position.x, ball.position.y)) < 5:
            return ball
    return None

def euclidean_distance(point1: Tuple[float, float], point2: Tuple[float, float]) -> float:
    # Unpack the points
    return math.dist(point1, point2)
def app(connect_to_robot: bool = False):

    tick_count = 0

    screen = __init__.screen
    robot = __init__.robot
    balls = __init__.balls
    walls = __init__.walls
    clock = __init__.clock
    cross = __init__.cross
    running = True
    first_iteration = True
    #goal = Goal(radius=1, position=Position(x=256, y=screen.get_height()/2))
    focused_ball: CircleObject = None
    if (connect_to_robot):
        
        transmission.connect()
        wp = WallPicker()
        goal = wp.pick_hole()

        wall_squares = [wp.pick_west_wall(), wp.pick_north_wall(),  wp.pick_east_wall(), wp.pick_south_wall()]
        
        walls = []
        left = Wall.create(wall_squares[0], WallPlacement.LEFT, danger_zone_size=5)
        walls.append(left)
        top = Wall.create(wall_squares[1], WallPlacement.TOP, danger_zone_size=5)
        walls.append(top)
        right = Wall.create(wall_squares[2], WallPlacement.RIGHT, danger_zone_size=5)
        walls.append(right)
        bot = Wall.create(wall_squares[3], WallPlacement.BOT, danger_zone_size=5)
        walls.append(bot)
        

        #Print til at indsætte i init. DO NOT DELETE OK THANK YOU
        print("left_wall_square = SquareObject.create_square(position=Position(x=" + str(left.position.x) + ", y=" + str(left.position.y) + "),\n" +
        "width=" + str(left.width) + ", height=" + str(left.height) + ", radians=" + str(left.radians) + ")")
        print("right_wall_square = SquareObject.create_square(position=Position(x=" + str(right.position.x) + ", y=" + str(right.position.y) + "),\n" +
        "width=" + str(right.width) + ", height=" + str(right.height) + ", radians=" + str(right.radians) + ")")
        print("top_wall_square = SquareObject.create_square(position=Position(x=" + str(top.position.x) + ", y=" + str(top.position.y) + "),\n" +
        "width=" + str(top.width) + ", height=" + str(top.height) + ", radians=" + str(top.radians) + ")")
        print("bot_wall_square = SquareObject.create_square(position=Position(x=" + str(bot.position.x) + ", y=" + str(bot.position.y) + "),\n" +
        "width=" + str(bot.width) + ", height=" + str(bot.height) + ", radians=" + str(bot.radians) + ")")



        rv = RoboVision(walls=walls, ai=True,
                        power=2)  # power: how strong the model should be (light(1), medium(2), heavy(3))
        cross_squares = wp.pick_cross()
        cross = Cross.create_cross_with_safe_zones(square_1=cross_squares[0], square_2=cross_squares[1], walls=walls,
                                                   safe_distance=20)

    while running:
        path = []
        i = 1
        found_robot_init = False
        if connect_to_robot:
            #if ai:
            while i < 100 and not found_robot_init:
                try:
                    balls, robot_square_object = rv.get_any_thing(min_count=0, max_count=20, tries=100, thing_to_get="all_balls")
                    robot_position: Position = robot_square_object.position
                    radians = rv.orientation
                    mode = robot.mode
                    previous_checkpoint = robot.prev_checkpoint
                    closest_obstacle = robot.closest_obstacle
                    robot = robot.create_robot(position=Position(x=robot_position.x, y=robot_position.y),
                                            width=130, height=130, radians=radians, suction_height=30, suction_width=30,
                                            suction_offset_y=80, previous_checkpoint=previous_checkpoint, mode=mode, closest_obstacle=closest_obstacle)
                except Exception as e:
                    print(str(e))
                finally:
                    found_robot_init = True
        if focused_ball and len(balls) > 0 and robot.mode != RobotMode.DANGER:
            temp_focused_ball = find_focused_ball(focused_ball, balls)
            if temp_focused_ball:
                focused_ball = temp_focused_ball
            else:
                focused_ball = sorted(balls, key=lambda ball: robot.calculate_speed_to_ball(ball))[0]

        if len(balls) > 0 and first_iteration:
            first_iteration = False
            focused_ball = balls[0]
        if balls == []:
            focused_ball = goal
            robot.mode = RobotMode.ENDPHASE
        elif robot.mode == RobotMode.ENDPHASE:
            robot.mode = RobotMode.SAFE

        if robot.prev_checkpoint.checkpoint_type != CheckpointType.GOAL:
            path, checkpoints = path_creation.create_path(focused_ball, robot, walls, cross)
            robot.checkpoints = checkpoints
        try:
            move: Move = path_follow.create_move(robot)
            path_follow.move_robot(move, robot, walls, balls, cross, connect_to_robot)
        except Exception:
            continue

        if connect_to_robot:
            transmission.send_command(move, robot.mode)
        print("Current mode: ", robot.mode)
        print("previous CheckpointType: ", robot.prev_checkpoint.checkpoint_type)
        print("Next CheckpointType: ", robot.checkpoints[0].checkpoint_type)
        print("Move: ", move)
        print("Robotposition: x=", robot.robot.position.x, "y= ", robot.robot.position.y)

        # Inserts a camera overlay instead of a background
        if connect_to_robot:
            frame = rv.get_flipped_frame()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = pygame.surfarray.make_surface(np.rot90(frame))
            screen.blit(frame, (0,0))

        # Update the games visual representation
        visualization.game(screen, robot, walls, balls, path, cross)

        # Tickrate, frames/sec.
        clock.tick(30) / 1000


        # Hello
        for event in pygame.event.get():
            if event.type == pygame.QUIT or move.latch == True:
                running = False

        print("balls: ", balls)


if __name__ == '__main__':
    app(False)
    pygame.quit()

