from typing import List, ForwardRef
from src.dto import CircleObject, SquareObject, Position
from pydantic import BaseModel



cross_1 = SquareObject.create_square(position=Position(x=240, y=550),
                    width=160, height=20, radians=0.785398)
cross_2 = SquareObject.create_square(position=Position(x=240, y=550),
                    width=160, height=20, radians=2.35619)

obstacles = [cross_1, cross_2]

balls = [CircleObject(radius=5, position=Position(x=124, y=534)),
         CircleObject(radius=5, position=Position(x=872, y=231)),
         CircleObject(radius=5, position=Position(x=512, y=333)),
         CircleObject(radius=5, position=Position(x=432, y=90)),
         CircleObject(radius=5, position=Position(x=144, y=666)),
         CircleObject(radius=5, position=Position(x=963, y=155)),
         CircleObject(radius=5, position=Position(x=1111, y=514)),
         CircleObject(radius=5, position=Position(x=893, y=625)),
         CircleObject(radius=5, position=Position(x=231, y=403)),]


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
(__name__== '__main__'):
    navAlgorithm(white_balls=balls,
                 orange_ball=CircleObject(radius=5, position=Position(x=127, y=344)),
                 egg=CircleObject(radius=5, position=Position(x=367, y=864)),
                 obstacles=obstacles)
"""