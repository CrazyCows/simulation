import gui.visualization as visualization
import transmission
from path import path_creation, path_follow
from dto.robot import Move, Checkpoint, RobotMode, CheckpointType
from dto.shapes import Position, CircleObject, Goal
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
    goal = Goal(radius=1, position=Position(x=230, y=screen.get_height()/2))
    #if connect_to_robot:
    #    transmission.connect()
        
    if (connect_to_robot):
        #wp = WallPicker()
        #wall_squares = [wp.pick_east_wall(), wp.pick_north_wall(),  wp.pick_west_wall(), wp.pick_south_wall()]
        rv = RoboVision(walls=walls)
        #cross_squares = wp.pick_cross()
        
        #walls = []
        #walls.append(Wall.create(wall_squares[0], WallPlacement.LEFT, danger_zone_size=100))
        #walls.append(Wall.create(wall_squares[1], WallPlacement.RIGHT, danger_zone_size=100))
        #walls.append(Wall.create(wall_squares[2], WallPlacement.TOP, danger_zone_size=100))
        #walls.append(Wall.create(wall_squares[3], WallPlacement.BOT, danger_zone_size=100))
        #cross = Cross.create_cross_with_safe_zones(square_1=cross_squares[0], square_2=cross_squares[1], walls=walls,
        #                                           safe_distance=20)
    


    while running:
        path = []
        if connect_to_robot:
            balls = rv.get_any_thing(min_count=0, max_count=20, tries=100, thing_to_get="orange_ball")
            if balls == []:
                #print("yes")
                balls = rv.get_any_thing(min_count=0, max_count=20, tries=100, thing_to_get="white_ball")
                #print(balls)
            robot_square_object = rv.get_any_thing(min_count=1, max_count=1, tries=200, thing_to_get="robot")
            
            robot_position = robot_square_object.position
            radians = robot_square_object.radians
            print(radians)
            robot = robot.create_robot(position=Position(x=robot_position.x, y=robot_position.y),
                                       width=30, height=30, radians=radians, suction_height=20, suction_width=20,
                                       suction_offset_y=25)
            #robot = robot.create_robot(position=Position(x=400, y=500),
            #    width=135, height=150, radians=3.14, suction_height=50, suction_width=25, suction_offset_y=105)
        if balls == [] or balls is None:
            continue
        #print(len(balls))
        # print(len(robot.collected_balls))
        # TODO: Implement the
        """
        if balls == []:
            balls.append(goal)
            if robot.mode != RobotMode.DEPOSIT:
                robot.mode = RobotMode.ENDPHASE
            # exit()
        # NOTE: This is jank as fuck... Fight me.
        elif ((isinstance(balls[0], Goal)) and len(balls) == 1):
            ""
            # print(len(balls))
        elif len(balls) > 1 and isinstance(balls[0], Goal):
            balls.remove(goal)
        """
        

        # TODO: Remove this(?) Will anyone get mad? Been here forever, doesn't do shit.
        def calculate_speed_to_ball(ball_start: CircleObject, ball_end: CircleObject):
            return dist((ball_start.position.x, ball_start.position.y), (ball_end.position.x, ball_end.position.y))

        if robot.mode != RobotMode.DANGER and robot.mode != RobotMode.DANGER_REVERSE:
            balls.sort(key=lambda ball: robot.calculate_speed_to_ball(ball))


        if robot.mode != RobotMode.STOP or robot.mode != RobotMode.STOP_DANGER:
            if robot.prev_checkpoint.checkpoint_type != CheckpointType.GOAL:
                path, checkpoints = path_creation.create_path(balls[0], robot, walls, cross)
                robot.checkpoints = checkpoints
            try:
                move: Move = path_follow.create_move(robot)
                path_follow.move_robot(move, robot, walls, balls, cross, connect_to_robot)
            except Exception as e:
                continue
        #print("Left: ", robot.distance_to_wall_left)
        #print("Right: ", robot.distance_to_wall_right)
        #print("Top: ", robot.distance_to_wall_top)
        #print("Bot: ", robot.distance_to_wall_bot)
        #print("Cross: ", robot.distance_to_cross)
        #print("Suck: ", move.suck)
        #print("Latch: ", move.latch)
        #print("Robot Mode:", robot.mode)
        #print("Checkpoint type:", robot.prev_checkpoint.checkpoint_type)
        if connect_to_robot:
            transmission.send_command(move)


        # NOTE: Updates the visual representation
        visualization.game(screen, robot, walls, balls, path, cross)

        # Tickrate, frames/sec.
        clock.tick(60) / 1000

        # Stop program
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False


if __name__ == '__main__':
    #pygame.init()
    app(True)
    pygame.quit()
