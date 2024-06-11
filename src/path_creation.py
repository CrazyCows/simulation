from typing import List, ForwardRef
from dto import CircleObject, SquareObject, Position, Player
from pydantic import BaseModel

def find_closest_ball(balls: List[CircleObject], player: Player) -> List[CircleObject]:
    balls.sort(key=lambda ball: player.calculate_distance_to_ball(ball))

def check_for_obstacles():
    ""




"""
Vertex = ForwardRef('Vertex')
class Vertex(BaseModel):
    position: Position
    connected_vertices: List[Vertex]



def navAlgorithm(white_balls: List[CircleObject],
                 orange_ball: CircleObject,
                 egg: CircleObject,
                 obstacles: List[SquareObject]):
    white_ball_vertices: List[Vertex] = [Vertex(position=ball.position, connected_vertices=[]) for ball in white_balls]
    orange_ball_vertex = Vertex(position=orange_ball.position, connected_vertices=[])
    egg_vertex = Vertex(position=egg.position, connected_vertices=[])
    obstacles_vertices: List[Vertex] = []
    for square in obstacles:
        obstacles_vertices.append(Vertex(position=Position(x=square.vertices[0][0], y=square.vertices[0][1]), connected_vertices=[]))
        obstacles_vertices.append(Vertex(position=Position(x=square.vertices[1][0], y=square.vertices[1][1]), connected_vertices=[]))

    for obs in obstacles_vertices:
        print(obs)



def point_to_point_nav(position1: Position, position2: Position, obstacles: List[SquareObject]):
    pass
"""


"""
(__name__== '__main__'):
    navAlgorithm(white_balls=balls,
                 orange_ball=CircleObject(radius=5, position=Position(x=127, y=344)),
                 egg=CircleObject(radius=5, position=Position(x=367, y=864)),
                 obstacles=obstacles)
"""