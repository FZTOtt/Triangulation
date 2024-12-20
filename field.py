import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from scipy.spatial import Delaunay
from point import Point
from obstacle import Obstacle

class Field:
    def __init__(self, start: Point, finish: Point, edges: list[Point], obstacles: list[Obstacle] = None, plot = False) -> None:
        self.start_point = start
        self.finish_point = finish
        self.edges = edges
        self.obstacles = obstacles
        self.points: list[Point] = []
        self.plot = plot

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
        if self.plot:
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
                G.add_edge(centroid, midpoints[0], weight=np.linalg.norm(np.array(centroid) - np.array(midpoints[0])))
                v.append(midpoints[0])

            if not self.is_edge_on_obstacle(p2, p3):
                G.add_node(midpoints[1])
                G.add_edge(centroid, midpoints[1], weight=np.linalg.norm(np.array(centroid) - np.array(midpoints[1])))
                v.append(midpoints[1])

            if not self.is_edge_on_obstacle(p1, p3):
                G.add_node(midpoints[2])
                G.add_edge(centroid, midpoints[2], weight=np.linalg.norm(np.array(centroid) - np.array(midpoints[2])))
                v.append(midpoints[2])

            for point in vertices:
                if not self.is_point_on_obstacle(point):
                    G.add_node(point)
                    G.add_edge(centroid, point, weight=np.linalg.norm(np.array(centroid) - np.array(point)))
                    v.append(point)

            for i in range(len(v)):
                for j in range(i + 1, len(v)):
                    G.add_edge(v[i], v[j], weight=np.linalg.norm(np.array(v[i]) - np.array(v[j])))

        G.add_node((self.start_point.x, self.start_point.y))
        G.add_node((self.finish_point.x, self.finish_point.y))

        return G

    def draw_graph(self, G, tri, points_array, shortest_path=None):
        pos = {node: node for node in G.nodes()}
        fig, ax = plt.subplots()

        for obstacle in self.obstacles:
            x_coords, y_coords = zip(*[(point.x, point.y) for point in obstacle.points])
            ax.fill(x_coords, y_coords, color=obstacle.color, alpha=0.5)

        ax.triplot(points_array[:, 0], points_array[:, 1], tri.simplices, color='gray', alpha=0.5)

        nx.draw(G, pos, with_labels=False, node_size=20, node_color='blue', ax=ax)
        ax.plot(self.start_point.x, self.start_point.y, 'go', label='Start')
        ax.plot(self.finish_point.x, self.finish_point.y, 'ro', label='Finish')

        if shortest_path:
            path_coords = np.array([pos[node] for node in shortest_path])
            ax.plot(path_coords[:, 0], path_coords[:, 1], 'r-', linewidth=2, label='Кратчайший путь')

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
