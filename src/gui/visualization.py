import pygame
from dto.robot import Robot
from dto.shapes import SquareObject, CircleObject, Position
from dto.obstacles import Cross
from typing import List


def game(screen: pygame.Surface, robot: Robot, obstacles: List[SquareObject], balls: List[CircleObject], calculated_path: List[SquareObject], cross: Cross):
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False

    #screen.fill("purple")

    for obstacle in obstacles:
        pygame.draw.polygon(screen, "red", obstacle.vertices)

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
        pygame.draw.circle(screen, "green", (ball.position.x, ball.position.y), ball.radius)

    for zone in cross.safe_zones:
        pygame.draw.circle(screen, "pink", (zone.x, zone.y), ball.radius)

    create_trail(robot.previous_path, color="white")

    pygame.display.flip()


