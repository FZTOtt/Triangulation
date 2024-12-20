import json
import time
from point import Point
from obstacle import Obstacle
from field import Field

PLOT = True

with open('2.json') as f:
    data = json.load(f)

def filter_by_type(data, type_name):
    return [item for item in data if item.get('type') == type_name]

info_elements = filter_by_type(data, 'info')
start_points = filter_by_type(data, 'startPoint')
end_points = filter_by_type(data, 'endPoint')
polygons = filter_by_type(data, 'polygon')

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

merged_obstacles = Obstacle.merge_intersecting_obstacles(obstacles)

field = Field(start, finish, edges_points, merged_obstacles)

if PLOT:
    field.draw_obstacles()
start_time = time.time()
tri, points_array = field.triangulate_free_space()

G = field.create_graph(tri, points_array)
shortest_path = field.find_shortest_path(G)
if PLOT:
    field.draw_graph(G, tri, points_array, shortest_path)

total_time = time.time() - start_time

# print("Кратчайший путь:", shortest_path)
print("время выполнения:", total_time)