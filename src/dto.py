from pydantic import BaseModel, confloat
from typing import Optional, List, Tuple
import math
import numpy as np



class Position(BaseModel):
    x: float
    y: float

class SquareObject(BaseModel):
    position: Position
    width: int
    height: int
    radians: float
    vertices: List[Tuple[float, float]]
    offset_x: int
    offset_y: int

    

    def update_square(self, position: Position, radians: float):
        # Apply offset to the position
        position_with_offset = Position(x=position.x + self.offset_x, y=position.y + self.offset_y)
        self.position = position_with_offset
        print(f'new pso: {position.y}')
        half_width = self.width / 2
        half_height = self.height / 2
        self.radians = radians
        self.vertices = [
            (position_with_offset.x - half_width, position_with_offset.y - half_height),
            (position_with_offset.x - half_width, position_with_offset.y + half_height),
            (position_with_offset.x + half_width, position_with_offset.y + half_height),
            (position_with_offset.x + half_width, position_with_offset.y - half_height)
        ]

        if radians:
            self.vertices = rotate_square(vertices=self.vertices, center=position, radians=radians)
            

    
    @classmethod
    def create_square(cls, position: Position, width: int, height: int, radians: float, offset_x=0, offset_y=0):
        position_with_offset = Position(x=position.x + offset_x, y=position.y + offset_y)

        half_width = width / 2
        half_height = height / 2
        vertices = [
            (position_with_offset.x - half_width, position_with_offset.y - half_height),
            (position_with_offset.x - half_width, position_with_offset.y + half_height),
            (position_with_offset.x + half_width, position_with_offset.y + half_height),
            (position_with_offset.x + half_width, position_with_offset.y - half_height)
        ]
        return cls(position=position, 
                   width=width, 
                   height=height, 
                   radians=radians, 
                   offset_x=offset_x, 
                   offset_y=offset_y, 
                   vertices=vertices)

class CircleObject(BaseModel):
    radius: int
    position: Position

    def contains_point(self, point: Position) -> bool:
        return math.hypot(self.position.x - point.x, self.position.y - point.y) <= self.radius
    

class Player(BaseModel):
    player: SquareObject
    suction: SquareObject
    collected_balls: List[CircleObject]
    obstacles_hit_list: List[SquareObject]
    obstacles_hit: int
    previous_path: List[Position]

    # TODO: Make move work correctly, such the player moves forward in a given direction. Currently player just moves forward.
    def move(self, distance, ddirection: float, obstacles: List[SquareObject]=[]):
        for obstacle in obstacles:
            if square_intersection(obstacle, self.player):
                self.obstacles_hit_list.append(obstacle)
                self.obstacles_hit += 1
                return

        dx = math.sin(ddirection) * distance
        dy = math.cos(ddirection) * distance

        self.previous_path.append(self.player.position)
        player_dx = self.player.position.x + dx
        player_dy = self.player.position.y + dy
        ddirection += self.player.radians
        print(ddirection)
        self.player.update_square(Position(x=player_dx, y=player_dy), ddirection)
        self.suction.update_square(Position(x=player_dx, y=player_dy), ddirection)


        
    def suck(self, balls: List[CircleObject]):
        for ball in balls:
            if circle_square_intersection(ball, self.suction):
                self.collected_balls.append(ball)

    @classmethod
    def create_player(cls, position: Position, width: int, height: int, radians: float, 
                suction_width: int, suction_height: int, suction_offset_x: int=0, suction_offset_y: int=0):

        if not 0 <= radians <= 2 * math.pi:
            raise Exception("Number must be within range 0 to 2 * pi")

        player=SquareObject.create_square(position=position, 
                                 width=width, 
                                 height=height, 
                                 radians=radians)
        suction=SquareObject.create_square(position=position, 
                                  width=suction_width, 
                                  height=suction_height, 
                                  radians=radians,
                                  offset_x=suction_offset_x,
                                  offset_y=suction_offset_y)
        collected_balls = []
        
        obstacles_hit_list = []
        obstacles_hit = 0
        previous_path = []
        return cls(player=player, suction=suction, collected_balls=collected_balls, 
                   obstacles_hit_list=obstacles_hit_list, obstacles_hit=obstacles_hit, previous_path=previous_path)


def rotate_square(vertices: List[Tuple[float, float]], center: Position, radians: float) -> List[Tuple[float, float]]:
    cx = center.x
    cy = center.y
    
    rotation_matrix = np.array([
        [np.cos(radians), -np.sin(radians)],
        [np.sin(radians),  np.cos(radians)]
    ])
    
    translated_vertices = vertices - np.array([cx, cy])
    
    rotated_vertices = np.dot(translated_vertices, rotation_matrix)
    
    rotated_vertices += np.array([cx, cy])

    return rotated_vertices

def square_intersection(square1: SquareObject, square2: SquareObject) -> bool:
    def project(vertices, axis):
        dots = [np.dot(vertex, axis) for vertex in vertices]
        return [min(dots), max(dots)]

    def overlap(proj1, proj2):
        return proj1[0] <= proj2[1] and proj2[0] <= proj1[1]

    def separating_axis_theorem(vertices1, vertices2):
        axes = []
        for i in range(len(vertices1)):
            p1 = vertices1[i]
            p2 = vertices1[(i + 1) % len(vertices1)]
            edge = np.subtract(p2, p1)
            normal = [-edge[1], edge[0]]
            axes.append(normal)
        for i in range(len(vertices2)):
            p1 = vertices2[i]
            p2 = vertices2[(i + 1) % len(vertices2)]
            edge = np.subtract(p2, p1)
            normal = [-edge[1], edge[0]]
            axes.append(normal)
        
        for axis in axes:
            proj1 = project(vertices1, axis)
            proj2 = project(vertices2, axis)
            if not overlap(proj1, proj2):
                return False
        return True
    return separating_axis_theorem(square1.vertices, square2.vertices)
        

def circle_intersection(circle1: CircleObject, circle2: CircleObject) -> bool:
    distance = math.hypot(circle1.position.x - circle2.position.x, circle1.position.y - circle2.position.y)
    return distance <= (circle1.radius + circle2.radius)


def circle_square_intersection(circle: CircleObject, square: SquareObject) -> bool:
    def point_in_polygon(point: Position, vertices: List[Tuple[float, float]]) -> bool:
        x, y = point.x, point.y
        n = len(vertices)
        inside = False
        px, py = vertices[0]
        for i in range(1, n + 1):
            vx, vy = vertices[i % n]
            if y > min(py, vy):
                if y <= max(py, vy):
                    if x <= max(px, vx):
                        if py != vy:
                            xinters = (y - py) * (vx - px) / (vy - py) + px
                        if px == vx or x <= xinters:
                            inside = not inside
            px, py = vx, vy
        return inside

    def circle_line_segment_intersection(circle: CircleObject, p1: Position, p2: Position) -> bool:
        cx, cy, r = circle.position.x, circle.position.y, circle.radius
        x1, y1 = p1.x, p1.y
        x2, y2 = p2.x, p2.y

        dx, dy = x2 - x1, y2 - y1
        fx, fy = x1 - cx, y1 - cy

        a = dx * dx + dy * dy
        b = 2 * (fx * dx + fy * dy)
        c = (fx * fx + fy * fy) - r * r

        discriminant = b * b - 4 * a * c
        if discriminant >= 0:
            discriminant = math.sqrt(discriminant)
            t1 = (-b - discriminant) / (2 * a)
            t2 = (-b + discriminant) / (2 * a)

            if 0 <= t1 <= 1 or 0 <= t2 <= 1:
                return True

        return False

    # Check if circle's center is inside the square
    if point_in_polygon(circle.position, square.vertices):
        return True

    # Check if any of the square's vertices are inside the circle
    for vertex in square.vertices:
        if circle.contains_point(Position(x=vertex[0], y=vertex[1])):
            return True

    # Check if the circle intersects with any of the square's edges
    for i in range(len(square.vertices)):
        p1 = square.vertices[i]
        p2 = square.vertices[(i + 1) % len(square.vertices)]
        if circle_line_segment_intersection(circle, Position(x=p1[0], y=p1[1]), Position(x=p2[0], y=p2[1])):
            return True

    return False

