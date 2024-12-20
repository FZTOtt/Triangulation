import json
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import random
from scipy.spatial import Delaunay

with open('1.json') as f:
    data = json.load(f)

def filter_by_type(data, type_name):
    return [item for item in data if item.get('type') == type_name]

info_elements = filter_by_type(data, 'info')
start_points = filter_by_type(data, 'startPoint')
end_points = filter_by_type(data, 'endPoint')
polygons = filter_by_type(data, 'polygon')

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

class Obstacle:
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
    obstacle = Obstacle(points)
    obstacles.append(obstacle)

class Field:
    def __init__(self, start: Point, finish: Point, edges: list[Point], obstacles: list[Obstacle] = None) -> None:
        self.start_point = start
        self.finish_point = finish
        self.edges = edges
        self.obstacles = obstacles
        self.points: list[Point] = []
        self.triangles: list[Triangle] = []

    def triangulate_free_space(self):
        all_points = []
        for obstacle in self.obstacles:
            all_points.extend([(point.x, point.y) for point in obstacle.points])

        field_boundary = [(0, 0), (100, 0), (100, 100), (0, 100)]
        all_points.extend(field_boundary)
        points_array = np.array(all_points)

        tri = Delaunay(points_array)
        fig, ax = plt.subplots()
        ax.triplot(points_array[:, 0], points_array[:, 1], tri.simplices)
        ax.plot(points_array[:, 0], points_array[:, 1], 'o')

        for obstacle in self.obstacles:
            x_coords, y_coords = zip(*[(point.x, point.y) for point in obstacle.points])
            ax.fill(x_coords, y_coords, color=obstacle.color, alpha=0.5)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.grid(True)
        plt.show()

        return tri, points_array

    def add_point(self, point: Point) -> None:
        self.points.append(point)

    def print(self) -> None:
        for point in self.points:
            point.print()

    def draw_obstacles(self):
        fig, ax = plt.subplots()

        ax.plot(self.start_point.x, self.start_point.y, 'go', label='Start')
        ax.plot(self.finish_point.x, self.finish_point.y, 'ro', label='Finish')

        for obstacle in self.obstacles:
            x_coords, y_coords = zip(*[(point.x, point.y) for point in obstacle.points])
            ax.fill(x_coords, y_coords, color=obstacle.color, alpha=0.5)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.legend()
        ax.grid(True)
        plt.show()

    def create_graph(self, tri, points_array):
        G = nx.Graph()

        for simplex in tri.simplices:
            p1 = (points_array[simplex[0], 0], points_array[simplex[0], 1])
            p2 = (points_array[simplex[1], 0], points_array[simplex[1], 1])
            p3 = (points_array[simplex[2], 0], points_array[simplex[2], 1])

            centroid = ((p1[0] + p2[0] + p3[0]) / 3, (p1[1] + p2[1] + p3[1]) / 3)
            G.add_node(centroid)

            midpoints = [
                ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2),
                ((p2[0] + p3[0]) / 2, (p2[1] + p3[1]) / 2),
                ((p3[0] + p1[0]) / 2, (p3[1] + p1[1]) / 2)
            ]

            for midpoint in midpoints:
                G.add_node(midpoint)
                G.add_edge(centroid, midpoint)

        return G

    def draw_graph(self, G):
        pos = {node: node for node in G.nodes()}
        nx.draw(G, pos, with_labels=False, node_size=20, node_color='blue')
        plt.show()

field = Field(start, finish, edges_points, obstacles)

field.print()
field.draw_obstacles()
tri, points_array = field.triangulate_free_space()

G = field.create_graph(tri, points_array)
field.draw_graph(G)
