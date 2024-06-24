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
    if connect_to_robot:
        transmission.connect()
    focused_ball: CircleObject = None
    if (connect_to_robot):
        #wp = WallPicker()
        #wall_squares = [wp.pick_east_wall(), wp.pick_north_wall(),  wp.pick_west_wall(), wp.pick_south_wall()]

        #walls = []
        #walls.append(Wall.create(wall_squares[0], WallPlacement.LEFT, danger_zone_size=5))
        #walls.append(Wall.create(wall_squares[1], WallPlacement.RIGHT, danger_zone_size=5))
        #walls.append(Wall.create(wall_squares[2], WallPlacement.TOP, danger_zone_size=5))
        #walls.append(Wall.create(wall_squares[3], WallPlacement.BOT, danger_zone_size=5))

        #for wall in walls:
        #    print(wall)


        """walls.append(Wall.create(
            SquareObject(position=Position(x=155.0, y=366.75), width=672, height=16, radians=4.737681204935092,
                         vertices=[(154.5001599156308, 30.655147079146616), (138.50527721581045, 31.059779528397996),
                                   (155.4998400843692, 702.8448529208533), (171.49472278418955, 702.4402204716021)],
                         offset_x=0, offset_y=0), placement=WallPlacement.LEFT, danger_zone_size=5))
        walls.append(Wall.create(
            SquareObject(position=Position(x=606.5, y=28.5),
                         width=914, height=17, radians=0.006564457128618842,
                        vertices=[(149.45404902404266, 23.000118502842287), (149.56564399374827, 39.999752221329764), (1063.5459509759573, 33.999881497157716), (1063.4343560062516, 17.000247778670236)], offset_x=0, offset_y=0), placement=WallPlacement.RIGHT, danger_zone_size=5))
        walls.append(Wall.create(
            SquareObject(position=Position(x=1069.25, y=360.75), width=660, height=15, radians=4.726024498886192,
                         vertices=[(1072.249721113274, 30.678414320101467), (1057.2511155469047, 30.88294075964285), (1066.250278886726, 690.8215856798986), (1081.2488844530953, 690.6170592403571)], offset_x=0, offset_y=0), placement=WallPlacement.TOP, danger_zone_size=5))
        walls.append(Wall.create(
            SquareObject(position=Position(x=618.25, y=696.75), width=911, height=17, radians=6.2798922345608545, vertices=[(162.78046085997272, 686.7500542215874), (162.72447872663582, 703.7499620448889), (1073.7195391400273, 706.7499457784126), (1073.7755212733641, 689.7500379551111)], offset_x=0, offset_y=0), placement=WallPlacement.BOT, danger_zone_size=5))"""



        rv = RoboVision(walls=walls, ai=True, power=3)  # power: how strong the model should be (light(1), medium(2), heavy(3))
        #cross_squares = wp.pick_cross()
        #cross = Cross.create_cross_with_safe_zones(square_1=cross_squares[0], square_2=cross_squares[1], walls=walls,
        #                                           safe_distance=20)
        """cross = Cross.create_cross_with_safe_zones(square_1=SquareObject(position=Position(x=278.0, y=149.25), width=23, height=102, radians=5.588447030982883, vertices=[(301.81485966751865, 102.70859414439232), (236.51605090174172, 181.06716466332466), (254.18514033248135, 195.79140585560768), (319.4839490982583, 117.43283533667535)], offset_x=0, offset_y=0),
                                                   square_2=SquareObject(position=Position(x=278.0, y=149.25), width=23, height=102, radians=0.8760580505981936, vertices=[231.46, 125.44]),
                                                   walls=walls)"""
        #for cross in cross_squares:
        #    print(cross)
        print("Here")


    while running:
        path = []
        ai: bool = True
        if connect_to_robot:
            #if ai:
            balls, robot_square_object = rv.get_any_thing(min_count=0, max_count=20, tries=100, thing_to_get="all_balls")
            """else:
                balls = rv.get_any_thing(min_count=0, max_count=20, tries=100, thing_to_get="orange_ball")
                if balls == []:
                    balls = rv.get_any_thing(min_count=0, max_count=20, tries=100, thing_to_get="white_ball")
                robot_square_object: SquareObject = rv.get_any_thing(min_count=1, max_count=1, tries=200, thing_to_get="robot")"""

            robot_position: Position = robot_square_object.position
            radians = rv.orientation
            robot = robot.create_robot(position=Position(x=robot_position.x, y=robot_position.y),
                                       width=135, height=150, radians=radians, suction_height=30, suction_width=30,
                                       suction_offset_y=80)
        #print(len(balls))
        # print(len(robot.collected_balls))
        # TODO: Implement the
        if balls == [] or (isinstance(balls[0], Goal)):
            balls.append(goal)
            print(type(balls[0]))
            if robot.mode != RobotMode.DEPOSIT:
                robot.mode = RobotMode.ENDPHASE
            # exit()
        else:
            robot.mode = RobotMode.SAFE
        def calculate_speed_to_ball(ball_start: CircleObject, ball_end: CircleObject):
            return dist((ball_start.position.x, ball_start.position.y), (ball_end.position.x, ball_end.position.y))

        if robot.mode != RobotMode.DANGER and robot.mode != RobotMode.DANGER_REVERSE:
            balls.sort(key=lambda ball: robot.calculate_speed_to_ball(ball))
        # Temp solution, just redrawing balls all da time
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
        #print("1   Previous CheckpointType:", robot.prev_checkpoint.checkpoint_type, "Next CheckpointType:", robot.checkpoints[0].checkpoint_type, "mode:", robot.mode)
        #print("2   dist to checkpoint: ", int(robot.calculate_dist_to_checkpoint(robot.checkpoints[0])))
        #print("3   robot radians says:", robot.robot.radians, "and path_follow says:", path_follow.calculate_radians_to_turn(robot))
        #print("4   yet_another_calculate_radians_to_turn", path_follow.yet_another_calculate_radians_to_turn(robot), "another_calculate_radians_to_turn", path_follow.another_calculate_radians_to_turn(robot))
        #print("5   THE MOVE IS:", move)
        if connect_to_robot:
            transmission.send_command(move)


        # NOTE: Updates the visual representation
        visualization.game(screen, robot, walls, balls, path, cross)

        # Tickrate, frames/sec.
        clock.tick(30) / 1000

        # Hello
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False


if __name__ == '__main__':
    # test_antons_code()
    pygame.init()
    # try:
    #     transmission.exit_functions()
    # except Exception as e:
    #     logging.error(e)
    app(True)
    pygame.quit()
