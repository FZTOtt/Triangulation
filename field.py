import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from scipy.spatial import Delaunay
from point import Point
from obstacle import Obstacle
from triangualtion import bowyer_watson
import matplotlib.path as mplPath

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
            # plt.show()

        return tri, points_array
    
    def tr_space(self):

        all_points = []
        for obstacle in self.obstacles:
            all_points.extend([(point.x, point.y) for point in obstacle.points])

        field_boundary = [(0, 0), (100, 0), (100, 100), (0, 100)]
        all_points.extend(field_boundary)
        all_points.append((self.start_point.x, self.start_point.y))
        all_points.append((self.finish_point.x, self.finish_point.y))
        points_array = np.array(all_points)
        points_list = [Point(x, y) for x, y in points_array]
        triangles = bowyer_watson(points_list)
        # plt.show()

        return triangles

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
                return obstacle
        return False
    
    def is_point_inside_obstacle(self, point):
        for obstacle in self.obstacles:
            poly = mplPath.Path([(p.x, p.y) for p in obstacle.points])
            if poly.contains_point(point):
                return obstacle
        return False
    
    def find_intersection_with_obstacle(self, p1, p2):
        """
        Находит пересечение отрезка p1p2 с границей препятствия.
        """
        intersections = []
        for obstacle in self.obstacles:
            for i in range(len(obstacle.points)):
                o1 = obstacle.points[i]
                o2 = obstacle.points[(i + 1) % len(obstacle.points)]
                intersection = self.line_intersection(p1, p2, o1, o2)
                if intersection:
                    intersections.extend(intersection)
        return intersections if intersections else None


    def line_intersection(self, p1, p2, q1, q2):
        """
        Находит точку пересечения двух отрезков (p1p2) и (q1q2).
        """
        def det(a, b, c, d):
            return a * d - b * c

        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = q1.x, q1.y
        x4, y4 = q2.x, q2.y

        denom = det(x1 - x2, y1 - y2, x3 - x4, y3 - y4)
        if abs(denom) < 1e-7:
            return None  # Отрезки параллельны

        px = det(det(x1, y1, x2, y2), x1 - x2, det(x3, y3, x4, y4), x3 - x4) / denom
        py = det(det(x1, y1, x2, y2), y1 - y2, det(x3, y3, x4, y4), y3 - y4) / denom

        # Проверка, что точка пересечения находится на обоих отрезках
        if (
            min(x1, x2) <= px <= max(x1, x2)
            and min(y1, y2) <= py <= max(y1, y2)
            and min(x3, x4) <= px <= max(x3, x4)
            and min(y3, y4) <= py <= max(y3, y4)
        ):
            return [(px, py)]
        return None
    
    def find_all_line_vertices(self, start, end):
        '''Находит пересечения с предятствиями и добавляет середины отрезков вне препятствий'''
        intersections = self.find_intersection_with_obstacle(start, end)
        points = []
        points.extend([start, end])
        if not intersections:
            return
        points.extend(intersections)
        points.sort(key=lambda p: ((p[0] - start[0]) ** 2 + (p[1] - start[1]) ** 2)) # отсортировали по прямой (p[0] - start[0]) ** 2 + (p[1] - start[1]) ** 2

        result = []

        for i in range(len(points)-1):

            current_point = points[i]
            next_point = points[i+1]
            midpoint = ((current_point[0] + next_point[0]) / 2, (current_point[1] + next_point[1]) / 2)

            if not self.is_point_inside_obstacle(midpoint) and not self.is_point_on_obstacle(midpoint):
                result.append(midpoint)

        return result


    def create_graph(self, tri, points_array):
        G = nx.Graph()

        for simplex in tri.simplices:
            p1 = (points_array[simplex[0], 0], points_array[simplex[0], 1])
            p2 = (points_array[simplex[1], 0], points_array[simplex[1], 1])
            p3 = (points_array[simplex[2], 0], points_array[simplex[2], 1])

            centroid = ((p1[0] + p2[0] + p3[0]) / 3, (p1[1] + p2[1] + p3[1]) / 3)
            v = []
            G.add_node(centroid)
            if not self.is_point_inside_obstacle(centroid):
                v.append(centroid)

            midpoints = [
                ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2),
                ((p2[0] + p3[0]) / 2, (p2[1] + p3[1]) / 2),
                ((p3[0] + p1[0]) / 2, (p3[1] + p1[1]) / 2)
            ]

            vertices = [p1, p2, p3]
            

            for i, (p_start, p_end) in enumerate([(p1, p2), (p2, p3), (p3, p1)]):
                if not self.is_edge_on_obstacle(p_start, p_end):
                    midpoint = midpoints[i]
                    if not self.is_point_inside_obstacle(midpoint):
                        G.add_node(midpoint)
                        v.append(midpoint)
                    else:
                        # найти пересечение триангуляции с препятствием и добавить точку середины отрезка
                        vert = self.find_all_line_vertices(p_start, p_end)
                        if vert:
                            for point in vert:
                                G.add_node(point)
                                v.append(point)

            # Добавляем вершины треугольника, если они не на препятствиях
            for point in vertices:
                if not self.is_point_on_obstacle(point) and not self.is_point_inside_obstacle(point):
                    G.add_node(point)
                    v.append(point)

            for i in range(len(v)):
                for j in range(i + 1, len(v)):
                    if not self.find_intersection_with_obstacle(v[i], v[j]):
                        G.add_edge(v[i], v[j], weight=np.linalg.norm(np.array(v[i]) - np.array(v[j])))

        G.add_node((self.start_point.x, self.start_point.y))
        G.add_node((self.finish_point.x, self.finish_point.y))

        return G

    def create_own_graph(self, triangles):
        G = nx.Graph()

        for triangle in triangles:
            # Получаем вершины треугольника
            p1, p2, p3 = triangle.points
            # Центроид треугольника
            centroid = ((p1.x + p2.x + p3.x) / 3, (p1.y + p2.y + p3.y) / 3)
            if not self.is_point_inside_obstacle(centroid):
                G.add_node(centroid)
            else:
                continue
            
            # Средние точки сторон
            midpoints = [
                ((p1.x + p2.x) / 2, (p1.y + p2.y) / 2),
                ((p2.x + p3.x) / 2, (p2.y + p3.y) / 2),
                ((p3.x + p1.x) / 2, (p3.y + p1.y) / 2)
            ]
            
            vertices = [(p1.x, p1.y), (p2.x, p2.y), (p3.x, p3.y)]
            v = []

            # Добавляем рёбра к средним точкам, если они не пересекают препятствия
            for i, (p_start, p_end) in enumerate([(p1, p2), (p2, p3), (p3, p1)]):
                if not self.is_edge_on_obstacle((p_start.x, p_start.y), (p_end.x, p_end.y)):
                    midpoint = midpoints[i]
                    if not self.is_point_inside_obstacle(midpoint):
                        G.add_node(midpoint)
                        G.add_edge(centroid, midpoint,
                                weight=np.linalg.norm(np.array(centroid) - np.array(midpoint)))
                        v.append(midpoint)

            # Добавляем вершины треугольника, если они не на препятствиях
            for point in vertices:
                if not self.is_point_on_obstacle(point) and not self.is_point_inside_obstacle(point):
                    G.add_node(point)
                    G.add_edge(centroid, point,
                            weight=np.linalg.norm(np.array(centroid) - np.array(point)))
                    v.append(point)

            # Связываем все точки треугольника друг с другом
            for i in range(len(v)):
                for j in range(i + 1, len(v)):
                    G.add_edge(v[i], v[j],
                            weight=np.linalg.norm(np.array(v[i]) - np.array(v[j])))

        # Добавляем стартовую и финишную точки
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

    def draw_new_graph(self, G, triangles, shortest_path=None):
        pos = {node: node for node in G.nodes()}
        fig, ax = plt.subplots()

        # Отрисовка препятствий
        for obstacle in self.obstacles:
            x_coords, y_coords = zip(*[(point.x, point.y) for point in obstacle.points])
            ax.fill(x_coords, y_coords, color=obstacle.color, alpha=0.5)

        # Отрисовка всех треугольников (Delaunay)
        for triangle in triangles:
            p1, p2, p3 = triangle.points
            x = [p1.x, p2.x, p3.x, p1.x]
            y = [p1.y, p2.y, p3.y, p1.y]
            ax.plot(x, y, color='gray', alpha=0.5)

        # Рисуем граф
        nx.draw(G, pos, with_labels=False, node_size=20, node_color='blue', ax=ax)

        # Начальная и конечная точки
        ax.plot(self.start_point.x, self.start_point.y, 'go', label='Start')
        ax.plot(self.finish_point.x, self.finish_point.y, 'ro', label='Finish')

        # Отрисовка кратчайшего пути
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
        try:
            path = nx.shortest_path(G, source=start_node, target=finish_node, weight='weight')
            return path
        except nx.NetworkXNoPath:
            print(f"Нет пути между {start_node} и {finish_node}.")
            return None
