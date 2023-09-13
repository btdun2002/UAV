import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
import time
import copy
from itertools import permutations
from threading import Thread
import csv

# Set the number of UAVs, radius, and must-pass UAVs
n_uav = 120
radius = 15
must_pass_uav = [15, 30, 45, 60, 75, 90, 105]
max_distance = 3 * radius
box_size = 100
infinity = (box_size**2 + box_size**2 + box_size**2)**(1 / 2) + 1
max_init_attempts = 100

# Generate random coordinates for UAVs
uav_coords = np.zeros((n_uav, 3))
for i in range(n_uav):
    if i == 0:
        uav_coords[i] = np.random.randint(0, box_size, size=(3, ))
    else:
        init_attempts = 1
        while init_attempts < max_init_attempts:
            temp_coords = np.random.randint(0, box_size, size=(3, ))
            distances = cdist(uav_coords[:i], [temp_coords])
            if np.any(distances <= max_distance):
                break
            init_attempts += 1
        else:
            print("Try again, failed to initialize UAV")
            sys.exit()
        uav_coords[i] = temp_coords

# Calculate distances between UAVs and set distances greater than 2 * radius to 0
distances = cdist(uav_coords, uav_coords)
distances[distances > 2 * radius] = 0

G_graph = []
for i in range(n_uav):
    G_node = []
    for j in range(n_uav):
        if distances[i, j] > 0:
            B_I = 1e9
            n_p = 0.8
            P_T_I = 0.1
            G_T_I = 20
            G_R_I = 25
            L_I = (4 * np.pi * distances[i, j] / 1.55e-6)**-2
            h = 6.626e-34
            f_l = 2e14
            B_N_I = 1e7

            weight = 1 / (B_I * np.log2(1 +
                                        (n_p * P_T_I * G_T_I * G_R_I * L_I) /
                                        (2 * h * f_l * B_N_I)))
            G_node.append((j, weight))
    G_graph.append(G_node)


def calculate_path_weight(graph, path):
    weight = 0

    for i in range(len(path) - 1):
        current_node = path[i]
        next_node = path[i + 1]

        # Tìm trọng số của cạnh nối giữa current_node và next_node
        for neighbor, edge_weight in graph[current_node]:
            if neighbor == next_node:
                weight += edge_weight
                break

    return weight


def greedy():
    start = time.perf_counter()
    must_pass_uav_set = copy.deepcopy(must_pass_uav)
    G_path = [0]
    while True:
        init_index = G_path[-1]
        if len(G_path) > 1:
            prev_index = G_path[-2]
        else:
            prev_index = G_path[-1]
        G_path.remove(G_path[-1])
        temp_path = [init_index]
        temp_node = 0
        considered_path_set = set(())
        considered_mp_uav = False
        while (considered_mp_uav == False):
            min_weight = infinity
            for edge in G_graph[init_index]:
                neighbor_index, neighbor_weight = edge
                if must_pass_uav_set != [] and neighbor_index in must_pass_uav_set:
                    temp_path.append(neighbor_index)
                    G_path += temp_path
                    must_pass_uav_set.remove(neighbor_index)
                    considered_mp_uav = True
                    break

                if must_pass_uav_set == [] and neighbor_index == n_uav - 1:
                    temp_path.append(neighbor_index)
                    G_path += temp_path
                    elapsed_time = time.perf_counter() - start
                    weight = calculate_path_weight(G_graph, G_path)
                    print("greedy" + "    " + str(weight) + "    " +
                          str(elapsed_time))
                    return "greedy", weight, elapsed_time

                if neighbor_weight < min_weight and (
                        init_index, neighbor_index
                ) not in considered_path_set and neighbor_index != prev_index:
                    temp_node = neighbor_index
                    min_weight = neighbor_weight
            if min_weight == infinity:
                temp_node = prev_index
            if considered_mp_uav == False:
                considered_path_set.add((init_index, temp_node))
                if (temp_node in temp_path):
                    temp_path = temp_path[:temp_path.index(temp_node)]
                temp_path.append(temp_node)
            init_index = temp_node


import heapq


def dijkstra(graph, start):
    distances = [infinity] * len(graph)
    distances[start] = 0
    previous_nodes = [-1] * len(graph)

    pq = [(0, start)]

    while pq:
        dist, node = heapq.heappop(pq)

        if dist > distances[node]:
            continue

        for neighbor, weight in graph[node]:
            distance = distances[node] + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = node
                heapq.heappush(pq, (distance, neighbor))

    return distances, previous_nodes


def shortest_path(graph, start, end):
    _, previous_nodes = dijkstra(graph, start)
    path = []
    current_node = end
    while current_node != -1:
        path.insert(0, current_node)
        current_node = previous_nodes[current_node]
    return path


def shortest():
    start = time.perf_counter()
    length = {}
    for node in range(len(G_graph)):
        distances, _ = dijkstra(G_graph, node)
        length[node] = distances

    G_p_nodes = [0] + must_pass_uav + [n_uav - 1]

    min_path = ()
    min_distance = infinity
    try:
        # Find the shortest path among all permutations of the must-pass UAVs
        for path in permutations(G_p_nodes):
            if path[0] == 0 and path[len(G_p_nodes) - 1] == n_uav - 1:
                # Calculate the total distance of the path
                distance = sum(length[path[i]][path[i + 1]]
                               for i in range(len(path) - 1))
                if distance < min_distance:
                    min_distance = distance
                    min_path = path
    except:
        print("Failed to connect must-pass UAVs")
        sys.exit()
    final = [0]
    for i in range(len(min_path) - 1):
        final.extend(shortest_path(G_graph, min_path[i], min_path[i + 1])[1:])
    elapsed_time = time.perf_counter() - start
    weight = calculate_path_weight(G_graph, final)
    print("shortest" + "    " + str(weight) + "    " + str(elapsed_time))
    return "shortest", weight, elapsed_time


if __name__ == '__main__':
    output_file = 'output.csv'

    with open(output_file, 'a', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        algorithm, weight, elapsed_time = greedy()
        csv_writer.writerow([algorithm, weight, elapsed_time])
        algorithm, weight, elapsed_time = shortest()
        csv_writer.writerow([algorithm, weight, elapsed_time])