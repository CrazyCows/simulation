import pygame
from dto.robot import Robot
from dto.shapes import SquareObject, CircleObject, Position
from dto.obstacles import Cross, Wall
from typing import List
from dto.robot import CheckpointType


def game(screen: pygame.Surface, robot: Robot, obstacles: List[Wall], balls: List[CircleObject], calculated_path: List[SquareObject], cross: Cross):
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False
    
    screen.fill("purple")
    for obstacle in obstacles:
        pygame.draw.polygon(screen, "orange", obstacle.danger_zone.vertices)
    for obstacle in obstacles:
        pygame.draw.polygon(screen, "red", obstacle.vertices)
    pygame.draw.polygon(screen, "orange", cross.square_1.danger_zone.vertices)
    pygame.draw.polygon(screen, "orange", cross.square_2.danger_zone.vertices)
    pygame.draw.polygon(screen, "red", cross.square_1.vertices)
    pygame.draw.polygon(screen, "red", cross.square_2.vertices)

    for path in calculated_path:
        pygame.draw.polygon(screen, "grey", path.vertices)

    pygame.draw.polygon(screen, "green", robot.robot.vertices)
    pygame.draw.polygon(screen, "yellow", robot.suction.vertices)
    pygame.draw.line(screen, "white", (robot.line.start_pos.x, robot.line.start_pos.y), (robot.line.end_pos.x, robot.line.end_pos.y))

    def create_trail(paths: List[Position], color: str):
        for i, position in enumerate(paths):
            if i % 6 == 0:
                if i % 4 == 0:
                    prev_pos = position

                current_pos_x = position.x
                current_pos_y = position.y
                prev_pos_x = prev_pos.x
                prev_pos_y = prev_pos.y
                pygame.draw.line(screen, color, (prev_pos_x, prev_pos_y), (current_pos_x, current_pos_y))

    for ball in balls:
        pygame.draw.circle(screen, "green", (ball.position.x, ball.position.y), 4)
        #TODO: Make elif
    for checkpoint in robot.checkpoints:
        print("The last loaded (checkpoint)", checkpoint)
        if checkpoint.checkpoint_type.value == CheckpointType.BALL.value:
            pygame.draw.circle(screen, "green", (checkpoint.x, checkpoint.y), 8)
            pygame.draw.circle(screen, "black", (checkpoint.x, checkpoint.y), 4)
        elif checkpoint.checkpoint_type.value == CheckpointType.SAFE_CHECKPOINT.value:
            pygame.draw.circle(screen, "pink", (checkpoint.x, checkpoint.y), 5)
        elif checkpoint.checkpoint_type.value == CheckpointType.GOAL.value:
            pygame.draw.circle(screen, "orange", (checkpoint.x, checkpoint.y), 5)
        elif checkpoint.checkpoint_type.value == CheckpointType.DANGER_CHECKPOINT.value:
            pygame.draw.circle(screen, "yellow", (checkpoint.x, checkpoint.y), 5)
        elif checkpoint.checkpoint_type.value == CheckpointType.DANGER_REVERSE_CHECKPOINT.value:
            pygame.draw.circle(screen, "blue", (checkpoint.x, checkpoint.y), 5)
        else:
            pygame.draw.circle(screen, "purple", (checkpoint.x, checkpoint.y), 5)


    for zone in cross.safe_zones:
        pygame.draw.circle(screen, "pink", (zone.x, zone.y), 5)


    create_trail(robot.previous_path, color="white")

    pygame.display.flip()


