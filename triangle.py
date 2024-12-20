import json
import numpy as np
import matplotlib.pyplot as plt
# from shapely.geometry import Point, LineString, Polygon
import networkx as nx
import random
from scipy.spatial import Delaunay

with open('1.json') as f:
    data = json.load(f)


def filter_by_type(data, type_name):
    return [item for item in data if item.get('type') == type_name]

# Пример использования функции
info_elements = filter_by_type(data, 'info')
start_points = filter_by_type(data, 'startPoint')
end_points = filter_by_type(data, 'endPoint')
polygons = filter_by_type(data, 'polygon')

# Вывод результатов
# print("Info elements:", info_elements)
# print("Start points:", start_points[0]) 
# print("End points:", end_points)
# print("Polygons:", polygons)

# print(len(polygons))
# print(polygons[0])

# считали все объекты: начальная точка, конечная и все препядствия
# Далее, согласно алгоритму нужно покрыть треугольниками всё пространство. То есть взять препядствие, пройтись по каждой вершине и соединить с вершинами всех других препядствий, не допустив
# пересечения линии с препядствиями. Наверняка можно создать что-то типа поля и там задать запретную зону 

class Point:

    def __init__(self, x=None, y=None, dict=None) -> None:
        if dict is not None:
            self.x = dict[0].get('x')
            self.y = dict[0].get('y')
        else:
            self.x = x
            self.y = y

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Point):
            return self.x == other.x and self.y == other.y
        return False

    def print(self) -> None:
        print(self.x, self.y)

class Obstecale:
    
    def __init__(self, points: list[Point]) -> None:
        self.points = points
        self.color = (random.random(), random.random(), random.random())
        
class Triangle:
    def __init__(self, p1: Point, p2: Point, p3: Point) -> None:
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3

    def print(self) -> None:
        print(f"Triangle: ({self.p1.x}, {self.p1.y}), ({self.p2.x}, {self.p2.y}), ({self.p3.x}, {self.p3.y})")

start = Point(dict=start_points)
finish = Point(dict=end_points)
edges = [[0, 0], [100, 0], [0, 100], [100, 100]]
edges_points = list(map(lambda coords: Point(x=coords[0], y=coords[1]), edges))
obstacles = []

for polygon in polygons:
    points = polygon.get('points')
    points = list(map(lambda point: Point(x=point.get('x'), y=point.get('y')), points))
    obstecale = Obstecale(points)
    obstacles.append(obstecale)
    # break

obstecale1 = Obstecale([Point(10,10), Point(15,20), Point(20,20)])
obstecale2 = Obstecale([Point(70,70), Point(75,80), Point(80,80)])
# obstacles = [obstecale1, obstecale2]

class Field: 
    def __init__(self, start: Point, finish: Point, edges: list[Point], obstacles: list[Obstecale] = None) -> None:
        self.start_point = start
        self.finish_point = finish
        self.edges = edges
        self.obstacles = obstacles
        self.points: list[Point] = []
        self.triangles: list[Triangle] = []

    # def triangulate_free_space(self):
    #     all_points = [(self.start_point.x, self.start_point.y), (self.finish_point.x, self.finish_point.y)]
    #     for obstacle in self.obstacles:
    #         all_points.extend([(point.x, point.y) for point in obstacle.points])

    #     field_boundary = [(0, 0), (10, 0), (10, 10), (0, 10)] 
    #     all_points.extend(field_boundary)

    #     points_array = np.array(all_points)

    #     tri = Delaunay(points_array)

    #     fig, ax = plt.subplots()
    #     ax.triplot(points_array[:, 0], points_array[:, 1], tri.simplices)
    #     ax.plot(points_array[:, 0], points_array[:, 1], 'o')

    #     for obstacle in self.obstacles:
    #         x_coords, y_coords = zip(*[(point.x, point.y) for point in obstacle.points])
    #         ax.fill(x_coords, y_coords, color=obstacle.color, alpha=0.5)

    #     ax.set_xlabel('X')
    #     ax.set_ylabel('Y')
    #     ax.grid(True)
    #     plt.show()

    def add_point(self, point: Point) -> None:
        self.points.append(point)

    def print(self) -> None:
        for point in self.points:
            point.print()

    def draw_obstacles(self):
        fig, ax = plt.subplots()
        
        # Отображение начальной и конечной точки
        ax.plot(self.start_point.x, self.start_point.y, 'go', label='Start')
        ax.plot(self.finish_point.x, self.finish_point.y, 'ro', label='Finish')

        # Отображение препятствий
        for obstacle in self.obstacles:
            x_coords, y_coords = zip(*[(point.x, point.y) for point in obstacle.points])
            ax.fill(x_coords, y_coords, color=obstacle.color, alpha=0.5)

        # Настройка осей и легенды
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.legend()
        ax.grid(True)
        plt.show()

    def draw_lines(self):
        fig, ax = plt.subplots()

        # Отображение начальной и конечной точки
        ax.plot(self.start_point.x, self.start_point.y, 'go', label='Start')
        ax.plot(self.finish_point.x, self.finish_point.y, 'ro', label='Finish')

        # Отображение препятствий
        for obstacle in self.obstacles:
            x_coords, y_coords = zip(*[(point.x, point.y) for point in obstacle.points])
            ax.fill(x_coords, y_coords, color=obstacle.color, alpha=0.5)

        # Отображение линий
        for line in self.lines:
            p1, p2 = line
            ax.plot([p1.x, p2.x], [p1.y, p2.y], color=(random.random(), random.random(), random.random()))

        # Настройка осей и легенды
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.legend()
        ax.grid(True)
        plt.show()

    def triangulate_free_space(self):
        all_points = []

        for obstacle in self.obstacles:
            all_points.extend(obstacle.points)

        all_points.extend(self.edges)
        self.lines = []
        for i in range(len(all_points)):
            for j in range(i + 1, len(all_points)):
                line = (all_points[i], all_points[j])
                if not self.line_intersects_obstacles(line):
                    self.add_line(line)

        return 
        self.triangles = []
        for i in range(len(self.lines)):
            for j in range(i + 1, len(self.lines)):
                for k in range(j + 1, len(self.lines)):
                    p1, p2 = self.lines[i]
                    p3, p4 = self.lines[j]
                    p5, p6 = self.lines[k]
                    if p2 == p3 and p4 == p5:
                        triangle = Triangle(p1, p2, p4)
                        if not self.triangle_intersects_obstacles(triangle):
                            self.triangles.append(triangle)

    def add_line(self, line: tuple[Point, Point]):
        # Проверяем, пересекает ли новая линия существующие линии
        for existing_line in self.lines:
            intersection = self.line_intersects_line(line[0], line[1], existing_line[0], existing_line[1])
            if intersection:
                # Если пересекает, делим существующую линию на две
                self.lines.remove(existing_line)
                self.lines.append((existing_line[0], intersection))
                self.lines.append((intersection, existing_line[1]))
                self.add_line([line[0], intersection])  # Повторяем проверку для новой линии
                self.add_line([intersection, line[1]])
                return
        # Если не пересекает, добавляем новую линию
        self.lines.append(line)

    def line_intersects_obstacles(self, line: tuple[Point, Point]) -> bool:
        # Проверяет, пересекает ли линия какое-либо препятствие
        for obstacle in self.obstacles:
            for i in range(len(obstacle.points)):
                edge_start = obstacle.points[i]
                edge_end = obstacle.points[(i + 1) % len(obstacle.points)]
                if self.line_intersects_line(line[0], line[1], edge_start, edge_end):
                    return True
        return False
    
    def triangle_intersects_obstacles(self, triangle: Triangle) -> bool:
        # Проверяет, пересекает ли треугольник какое-либо препятствие
        for obstacle in self.obstacles:
            for i in range(len(obstacle.points)):
                edge_start = obstacle.points[i]
                edge_end = obstacle.points[(i + 1) % len(obstacle.points)]
                if (self.line_intersects_line(edge_start, edge_end, triangle.p1, triangle.p2) or
                    self.line_intersects_line(edge_start, edge_end, triangle.p2, triangle.p3) or
                    self.line_intersects_line(edge_start, edge_end, triangle.p3, triangle.p1)):
                    return True
        return False
    
    def line_intersects_line(self, p1: Point, p2: Point, p3: Point, p4: Point) -> Point:
        a1 = p2.y - p1.y
        b1 = p1.x - p2.x
        c1 = a1 * p1.x + b1 * p1.y

        a2 = p4.y - p3.y
        b2 = p3.x - p4.x
        c2 = a2 * p3.x + b2 * p3.y

        # Определитель системы
        det = a1 * b2 - a2 * b1

        if det == 0:
            # Прямые параллельны
            return None

        # Найдем точку пересечения
        x = (b2 * c1 - b1 * c2) / det
        y = (a1 * c2 - a2 * c1) / det

        # Проверим, лежит ли точка на обоих отрезках
        if (min(p1.x, p2.x) <= x <= max(p1.x, p2.x) and
            min(p1.y, p2.y) <= y <= max(p1.y, p2.y) and
            min(p3.x, p4.x) <= x <= max(p3.x, p4.x) and
            min(p3.y, p4.y) <= y <= max(p3.y, p4.y)):
            point = Point(x,y)
            if point == p1 or point == p2 or point == p3 or point == p4:
                return None
            return Point(x,y)
        else:
            return None

field = Field(start, finish, edges_points, obstacles)

field.print()
field.draw_obstacles()
field.triangulate_free_space()
field.draw_lines()


for polygon in polygons:
    current_points = polygon.get('points', [])
