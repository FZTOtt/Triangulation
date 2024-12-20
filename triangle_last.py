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
        all_points.append((self.start_point.x, self.start_point.y))
        all_points.append((self.finish_point.x, self.finish_point.y))
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

    def is_edge_on_obstacle(self, p1, p2):
        for obstacle in self.obstacles:
            obstacle_edges = [(obstacle.points[i].x, obstacle.points[i].y, obstacle.points[(i+1)%len(obstacle.points)].x, obstacle.points[(i+1)%len(obstacle.points)].y) for i in range(len(obstacle.points))]
            for edge in obstacle_edges:
                if (p1[0] == edge[0] and p1[1] == edge[1] and p2[0] == edge[2] and p2[1] == edge[3]) or (p1[0] == edge[2] and p1[1] == edge[3] and p2[0] == edge[0] and p2[1] == edge[1]):
                    return True
        return False

    def is_point_on_obstacle(self, point):
        for obstacle in self.obstacles:
            if point in [(p.x, p.y) for p in obstacle.points]:
                return True
        return False

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

            vertices = [p1, p2, p3]
            v = []

            if not self.is_edge_on_obstacle(p1, p2):
                G.add_node(midpoints[0])
                G.add_edge(centroid, midpoints[0])
                v.append(midpoints[0])

            if not self.is_edge_on_obstacle(p2, p3):
                G.add_node(midpoints[1])
                G.add_edge(centroid, midpoints[1])
                v.append(midpoints[1])

            if not self.is_edge_on_obstacle(p1, p3):
                G.add_node(midpoints[2])
                G.add_edge(centroid, midpoints[2])
                v.append(midpoints[2])

            for point in vertices:
                if not self.is_point_on_obstacle(point):
                    G.add_node(point)
                    G.add_edge(centroid, point)
                    v.append(point)

            # Add edges between all nodes in the triangle
            for i in range(len(v)):
                for j in range(i + 1, len(v)):
                    G.add_edge(v[i], v[j])

        G.add_node((self.start_point.x, self.start_point.y))
        G.add_node((self.finish_point.x, self.finish_point.y))

        return G

    def draw_graph(self, G):
        pos = {node: node for node in G.nodes()}
        fig, ax = plt.subplots()

        # Draw obstacles
        for obstacle in self.obstacles:
            x_coords, y_coords = zip(*[(point.x, point.y) for point in obstacle.points])
            ax.fill(x_coords, y_coords, color=obstacle.color, alpha=0.5)

        # Draw graph
        nx.draw(G, pos, with_labels=False, node_size=20, node_color='blue', ax=ax)
        ax.plot(self.start_point.x, self.start_point.y, 'go', label='Start')
        ax.plot(self.finish_point.x, self.finish_point.y, 'ro', label='Finish')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.legend()
        ax.grid(True)
        plt.show()

    def find_shortest_path(self, G):
        start_node = (self.start_point.x, self.start_point.y)
        finish_node = (self.finish_point.x, self.finish_point.y)
        path = nx.shortest_path(G, source=start_node, target=finish_node, weight='weight')
        return path

field = Field(start, finish, edges_points, obstacles)

field.print()
field.draw_obstacles()
tri, points_array = field.triangulate_free_space()

G = field.create_graph(tri, points_array)
field.draw_graph(G)

# Find the shortest path
shortest_path = field.find_shortest_path(G)
print("Shortest path:", shortest_path)
