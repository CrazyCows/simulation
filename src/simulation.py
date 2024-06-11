import pygame
from dto import Position, Input, Player, SquareObject, CircleObject
from typing import List


def game(screen: pygame.Surface, player: Player, obstacles: List[SquareObject], balls: List[CircleObject], calculated_path: List[SquareObject]):
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False

    screen.fill("purple")

    for obstacle in obstacles:
        pygame.draw.polygon(screen, "red", obstacle.vertices)

    for ball in balls:
        pygame.draw.circle(screen, "green", (ball.position.x, ball.position.y), ball.radius)
    
    for path in calculated_path:
        pygame.draw.polygon(screen, "grey", path.vertices)

    pygame.draw.polygon(screen, "green", player.player.vertices)
    pygame.draw.polygon(screen, "yellow", player.suction.vertices)
    pygame.draw.line(screen, "white", (player.line.start_pos.x, player.line.start_pos.y), (player.line.end_pos.x, player.line.end_pos.y))

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
    
    create_trail(player.previous_path, color="white")

    pygame.display.flip()


# NOTE: Function is just for testing. Is not used. Can be ignored.
def create_manual_inputs(keys):
    """
        This function is in case we want to control the robot manually.
    """
    inputs = []
    if keys[pygame.K_w] or "W" in keys:
        inputs.append(Input.FORWARD)
    if keys[pygame.K_s] or "S" in keys:
        inputs.append(Input.BACKWARD)
    if keys[pygame.K_a] or "A" in keys:
        inputs.append(Input.LEFT)
    if keys[pygame.K_d] or "D" in keys:
        inputs.append(Input.RIGHT)
    if keys[pygame.K_SPACE] or "SPACE" in keys:
        inputs.append(Input.SUCK)
    return inputs
