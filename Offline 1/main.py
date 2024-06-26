from queue import PriorityQueue
import math


class Graph:
    def __init__(self):
        self.graph = dict()

    def add_edge(self, source, destination, distance):
        if source not in self.graph:
            self.graph[source] = []
        if destination not in self.graph:
            self.graph[destination] = []

        self.graph[source].append([destination, int(distance)])
        self.graph[destination].append([source, int(distance)])

    def print_graph(self):
        for key in self.graph:
            print(f"{key}\t->{self.graph[key]}")

    def __getPath(self, parent, node, g_scores):
        path = [node]
        total_cost = 0
        while parent[node] is not None:
            total_cost += g_scores[node]
            node = parent[node]
            path.append(node)
        return list(reversed(path)), total_cost

    def dijkstra(self, start, goal):
        pq = PriorityQueue()
        pq.put((0, start))

        parent = {start: None}
        visited = set()
        distances = {node: float('inf') for node in self.graph}
        distances[start] = 0

        while not pq.empty():
            current_node = pq.get()[1]

            if current_node == goal:
                return self.__getPath(parent, current_node, distances)
            if current_node in visited:
                continue
            visited.add(current_node)
            for neighbor in self.graph[current_node]:
                alt_distance = distances[current_node] + neighbor[1]
                if alt_distance < distances[neighbor[0]]:
                    distances[neighbor[0]] = alt_distance
                    parent[neighbor[0]] = current_node
                    pq.put((distances[neighbor[0]], neighbor[0]))

        return None, float('inf')  # No path found

    def goal_vertices(self, goal, a, b, c):
        with open("Coordinates.csv","r") as f:
            lines = f.readlines()
        for l in lines[1:]:
            node, x, y, z = l.strip().split(",")
            if node == goal:
                return float(x), float(y), float(z)


    def AStar(self, start, goal, heuristic):
        pq = PriorityQueue()
        f_score = {start: 0 + heuristic[start]}
        pq.put((f_score[start], start))

        parent = {start: None}
        visited = set()
        g_score = {start: 0}

        while not pq.empty():
            current_node = pq.get()[1]

            if current_node == goal:
                return self.__getPath(parent, current_node, g_score)
            if current_node in visited:
                continue
            visited.add(current_node)
            for neighbor in self.graph[current_node]:
                if neighbor[0] not in visited:
                    parent[neighbor[0]] = current_node
                    g_score[neighbor[0]] = g_score[parent[neighbor[0]]] + neighbor[1]
                    f_score[neighbor[0]] = g_score[neighbor[0]] + heuristic[neighbor[0]]
                    pq.put((f_score[neighbor[0]], neighbor[0]))


g = Graph()
with open("distances.csv", "r") as f:
    lines = f.readlines()
    for l in lines:
        source, destination, distance = l.split(",")
        g.add_edge(source, destination, distance)
g.print_graph()

start = 'Sun'
goal = 'Upsilon Andromedae'
#start = 'TRAPPIST-1'
#goal = '55 Cancri'
#start = input("Enter source name: ")
#goal = input("Enter Destination name: ")

vala, valb, valc = g.goal_vertices(goal,0,0,0)
hn = dict()


with open("Coordinates.csv", "r") as f:
    lines = f.readlines()
for l in lines[1:]:
    node, x, y, z = l.strip().split(",")
    x, y, z = map(float, (x, y, z))
    dist = math.sqrt((x - vala) ** 2 + (y - valb) ** 2 + (z + -valc) ** 2)
    hn[node] = int(dist)


print()
#print(hn[goal])
path_AStar, cost_AStar = g.AStar(start, goal, hn)
print(f"A* Path: {path_AStar}, Total Cost: {cost_AStar}")

path_dijkstra, cost_dijkstra = g.dijkstra(start, goal)
print(f"Dijkstra Path: {path_dijkstra}, Total Cost: {cost_dijkstra}")

# Compare paths and costs
if path_AStar == path_dijkstra and cost_AStar == cost_dijkstra:
    print("Both A* and Dijkstra algorithms found the same path and total cost.")
else:
    print("A* and Dijkstra algorithms found different paths and/or total costs.")
