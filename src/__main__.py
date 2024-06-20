import gui.visualization as visualization
import transmission
from path import path_creation, path_follow
from dto.robot import Move, Checkpoint, Phase, CheckpointType
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
    balls = __init__.balls
    walls = __init__.walls
    clock = __init__.clock
    cross = __init__.cross
    running = True
    #if connect_to_robot:
    #    transmission.connect

    # TODO: The robot phase is currently set to goal phase for testing by Lucas
    #robot.phase = Phase.GOAL

    if (connect_to_robot):
        wp = WallPicker()
        walls = [wp.pick_east_wall(), wp.pick_north_wall(), wp.pick_south_wall(), wp.pick_west_wall()]
        rv = RoboVision()
        cross_squares = wp.click_cross()
        #print("Here")
        cross = Cross.create_cross_with_safe_zones(square_1=cross_squares[0], square_2=cross_squares[1], walls=walls, safe_distance=20)
    while running:
        if connect_to_robot:
            #print("WHY THE FUCK AM I RUNNING?")
            balls = RoboVision().get_any_thing(min_count=0, max_count=20, tries=100, thing_to_get="white_ball")
            temprobo = RoboVision().get_any_thing(min_count=1, max_count=1, tries=200, thing_to_get="robot")

            robot_position = temprobo.position
            #print(robot_position)
            radians = temprobo.radians
            robot = robot.create_robot(position=Position(x=robot_position.x, y=robot_position.y),
                                       width=30, height=30, radians=radians, suction_height=20, suction_width=20,
                                       suction_offset_y=25)


        # TODO: This is currently setup to only work with visualization tool. The above connect_to_robot should be merged in with the current phases below

        # The calibration phase: For calibrating the robot before it runs
        if robot.phase.value == Phase.CALIBRATE.value:
            print("Calibartion phase is currently running")

        # TODO: These phases are currently combined into one, but should be separated into two different phases
        # The pickup phase: For picking up balls and running its main sequence
        if robot.phase.value == Phase.PICKUP.value:# or robot.phase.value == Phase.GOAL.value:
            # Temp solution, just redrawing balls all da time
            path, checkpoints = path_creation.create_path(balls, robot, walls, cross)
            checkpoints = [Checkpoint(x=ball.position.x, y=ball.position.y, checkpointType=CheckpointType.BALL.value) for ball in balls]

            goal_position_left = Position(x=walls[0].position.x + 50, y=walls[1].height / 2)
            goal_position_right = Position(x=walls[1].position.x - 50, y=walls[1].height / 2)
            goal_left = Checkpoint(x=goal_position_left.x, y=goal_position_left.y, checkpointType=CheckpointType.GOAL.value)
            goal_right = Checkpoint(x=goal_position_right.x, y=goal_position_right.y, checkpointType=CheckpointType.GOAL.value)
            checkpoints.append(goal_left)
            checkpoints.append(goal_right)

            robot.checkpoints = checkpoints
            try:
                move: Move = path_follow.create_move(robot, walls)
                path_follow.move_robot(move, robot, walls, balls, cross, connect_to_robot)
            except Exception as e:
                continue

        # The goal phase: The goal sequence
        #if robot.phase.value == Phase.GOAL.value:
        #    print("Goal phase is currently running")

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
