from shapely.geometry import Polygon
from shapely.ops import unary_union
from obstacle import Obstacle, Point

def merge_all(obstacles):
    """
    Объединяет все пересекающиеся препятствия.
    """
    merged_obstacles = obstacles.copy()
    idx = 0
    while idx != len(merged_obstacles):
        current_obstacle: Obstacle = merged_obstacles[idx]
        if current_obstacle is None:
            idx += 1
            continue
        i = idx + 1
        while i != len(merged_obstacles):
            if (merged_obstacles[i] != None):
                obst = current_obstacle.merge_with(merged_obstacles[i])
                if isinstance(obst, tuple):
                    pass
                else:
                    merged_obstacles[idx] = obst
                    current_obstacle = obst
                    merged_obstacles[i] = None
            else:
                pass
            i += 1
        idx += 1
            
    merged_obstacles = [obstacle for obstacle in merged_obstacles if obstacle is not None]
    return merged_obstacles


def merge_all_obstacles_shapely(obstacles):
    """
    Объединяет все пересекающиеся препятствия с использованием библиотеки shapely.
    """
    polygons = [Polygon([(point.x, point.y) for point in obstacle.points]) for obstacle in obstacles]
    merged_polygon = unary_union(polygons)

    merged_obstacles = []
    if merged_polygon.geom_type == 'MultiPolygon':
        for polygon in merged_polygon.geoms:
            merged_obstacles.append(Obstacle([Point(x, y) for x, y in polygon.exterior.coords]))
    else:
        merged_obstacles.append(Obstacle([Point(x, y) for x, y in merged_polygon.exterior.coords]))

    return merged_obstacles