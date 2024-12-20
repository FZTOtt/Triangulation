import random
import matplotlib.pyplot as plt
from math import atan2, pi
from point import Point


class Obstacle:
    def __init__(self, points: list[Point]) -> None:
        self.points = self.ensure_clockwise(points)
        self.color = (random.random(), random.random(), random.random())

    @staticmethod
    def ensure_clockwise(points: list[Point]) -> list[Point]:
        """
        Упорядочивает точки многоугольника по часовой стрелке.
        """
        # Найти центр многоугольника
        center_x = sum(p.x for p in points) / len(points)
        center_y = sum(p.y for p in points) / len(points)

        # Сортировать точки по углу относительно центра
        def angle(p):
            return atan2(p.y - center_y, p.x - center_x)

        sorted_points = sorted(points, key=angle)

        # Проверка направления обхода
        if Obstacle.is_clockwise(sorted_points):
            return sorted_points
        else:
            return sorted_points[::-1]

    @staticmethod
    def is_clockwise(points: list[Point]) -> bool:
        """
        Проверяет, идут ли точки по часовой стрелке.
        """
        sum_angles = 0
        for i in range(len(points)):
            p1 = points[i]
            p2 = points[(i + 1) % len(points)]
            sum_angles += (p2.x - p1.x) * (p2.y + p1.y)
        return sum_angles > 0

    def plot(self):
        """
        Визуализация многоугольника.
        """
        x = [p.x for p in self.points]
        y = [p.y for p in self.points]
        plt.fill(x, y, color=self.color, alpha=0.5)
        plt.plot(x, y, color='black')

    @staticmethod
    def line_intersection(p1, p2, p3, p4):
        """
        Находит точку пересечения двух отрезков (p1, p2) и (p3, p4).
        Возвращает None, если отрезки не пересекаются.
        """
        def ccw(A, B, C):
            return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x)

        def intersect(A, B, C, D):
            return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

        if intersect(p1, p2, p3, p4):
            # Найти точку пересечения
            xdiff = (p1.x - p2.x, p3.x - p4.x)
            ydiff = (p1.y - p2.y, p3.y - p4.y)

            def det(a, b):
                return a[0] * b[1] - a[1] * b[0]

            div = det(xdiff, ydiff)
            if div == 0:
                return None

            d = (det(*[(p1.x, p1.y), (p2.x, p2.y)]), det(*[(p3.x, p3.y), (p4.x, p4.y)]))
            x = det(d, xdiff) / div
            y = det(d, ydiff) / div
            return Point(x, y)
        return None

    def find_intersections(self, other: 'Obstacle') -> list[Point]:
        """
        Находит точки пересечения двух препятствий.
        """
        intersections = []
        for i in range(len(self.points)):
            p1 = self.points[i]
            p2 = self.points[(i + 1) % len(self.points)]
            for j in range(len(other.points)):
                p3 = other.points[j]
                p4 = other.points[(j + 1) % len(other.points)]
                intersection = Obstacle.line_intersection(p1, p2, p3, p4)
                if intersection:
                    intersections.append(intersection)
        return intersections
    
    def define_arrays(self, other: 'Obstacle', intersections: list[Point]):
        """
        Формирует два списка точек с учетом сортировки по часовой стрелке и добавления точек пересечения.
        """
        self_points = self.points + intersections
        self_points = self.ensure_clockwise(self_points)

        other_points = other.points + intersections
        other_points = self.ensure_clockwise(other_points)

        return self_points, other_points
    
    @staticmethod
    def is_point_inside(point: Point, polygon: list[Point]) -> bool:
        """
        Проверяет, находится ли точка внутри многоугольника.
        """
        def is_left(p0, p1, p2):
            return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y)

        wn = 0  # winding number
        n = len(polygon)
        for i in range(n):
            if polygon[i].y <= point.y:
                if polygon[(i + 1) % n].y > point.y:
                    if is_left(polygon[i], polygon[(i + 1) % n], point) > 0:
                        wn += 1
            else:
                if polygon[(i + 1) % n].y <= point.y:
                    if is_left(polygon[i], polygon[(i + 1) % n], point) < 0:
                        wn -= 1
        return wn != 0
    
    def merge_with(self, other: 'Obstacle') -> 'Obstacle' or tuple['Obstacle', 'Obstacle']:
        """
        Объединяет два препятствия в одно, если они пересекаются, иначе возвращает два препятствия.
        """
        intersections = self.find_intersections(other)
        if not intersections:
            # Если нет пересечений, возвращаем два препятствия
            return self, other
        self_points, other_points = self.define_arrays(other, intersections)

        merged_points = []
        idx = 0 
        first_outher_idx = -1
        for point in self_points:
            if point in self.points:
                # определи находится ли эта точка внутри препятствия other
                if not Obstacle.is_point_inside(point, other.points):
                    # нашли первую точку снаружи второго препятствия, Значит можно определить первую точку выхода
                    if idx != 0:
                        first_outher_idx = idx - 1
                    break
            idx += 1

        # теперь надо начать с начала списка (если first_outher_idx == -1) или с перой точки выхода
        idx_start = idx
        if first_outher_idx != -1:
            idx_start = first_outher_idx
        self_flag = True
        merged_points.append(self_points[idx_start])
        current_idx = idx_start + 1
        while True:
            if self_flag:
                if self_points[current_idx] in self.points:
                    merged_points.append(self_points[current_idx])
                    current_idx = (current_idx + 1) % len(self_points)
                else:
                    merged_points.append(self_points[current_idx])
                    current_point = self_points[current_idx]
                    other_idx = other_points.index(current_point)
                    self_flag = False
                    current_idx = (other_idx + 1) % len(other_points)
            else:
                if other_points[current_idx] in other.points:
                    merged_points.append(other_points[current_idx])
                    current_idx = (current_idx + 1) % len(other_points)
                else:
                    merged_points.append(other_points[current_idx])
                    # здесь надо найти эту точку пересечения в self_points
                    current_point = other_points[current_idx]
                    self_idx = self_points.index(current_point)
                    self_flag = True
                    current_idx = (self_idx + 1) % len(self_points)
            
            if merged_points[0] == merged_points[-1]:
                break

        if merged_points[0] == merged_points[-1]:
            merged_points.pop()

        return Obstacle(merged_points)
