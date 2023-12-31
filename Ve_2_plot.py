import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
import time
import copy
from itertools import permutations
import csv
from concurrent.futures import ThreadPoolExecutor

# Set the number of UAVs, radius, and must-pass UAVs
n_uav = 120
radius = 15
must_pass_uav = [60, 70, 80, 90, 100, 110]
max_distance = 3 * radius
box_size = 100
infinity = 3 * box_size**2
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
distances[distances > 1.9 * radius] = 0

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


def calculate_weight(path):
    weight = 0
    for i in range(len(path) - 1):
        for edge in G_graph[path[i]]:
            if edge[0] == path[i + 1]:
                weight += edge[1]
    return weight


def greedy():
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
                    return G_path

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


def D_final_sequence(previous_nodes, start, b):
    path = []
    current_node = b

    while current_node != start:
        path.insert(0, current_node)
        current_node = previous_nodes[current_node]

    path.insert(0, start)

    return path


def shortest():
    D_lengths = {}
    D_sequences = {}
    for node in [0] + must_pass_uav + [n_uav - 1]:
        distances, previous_nodes = dijkstra(G_graph, node)
        D_lengths[node] = distances
        D_sequences[node] = previous_nodes

    min_path = ()
    min_cost = infinity
    try:
        for permutation in permutations(must_pass_uav):
            permutation = [0] + list(permutation) + [n_uav - 1]
            cost = sum(D_lengths[permutation[i]][permutation[i + 1]]
                       for i in range(len(permutation) - 1))
            if cost < min_cost:
                min_cost = cost
                min_path = permutation
    except:
        print("Failed to connect must-pass UAVs")
        sys.exit()
    final = [0]
    for i in range(len(min_path) - 1):
        final.extend(
            D_final_sequence(D_sequences[min_path[i]], min_path[i],
                             min_path[i + 1])[1:])
    return final


def plot_path(path, title):
    colors = [
        'blue' if i == 0 else
        'red' if i == (n_uav -
                       1) else 'orange' if i in must_pass_uav else 'green'
        for i in range(n_uav)
    ]
    # Plot the UAVs and the path
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')

    for i in range(n_uav):
        ax.scatter(uav_coords[i, 0],
                   uav_coords[i, 1],
                   uav_coords[i, 2],
                   c=colors[i],
                   marker='o')

    start_node = path[0]
    end_node = path[-1]
    ax.text(uav_coords[start_node, 0],
            uav_coords[start_node, 1],
            uav_coords[start_node, 2],
            f'Start',
            color='black',
            fontsize=10,
            ha='center',
            va='bottom')
    ax.text(uav_coords[end_node, 0],
            uav_coords[end_node, 1],
            uav_coords[end_node, 2],
            f'End',
            color='black',
            fontsize=10,
            ha='center',
            va='bottom')

    for i in range(len(path) - 1):
        u, v = path[i], path[i + 1]
        ax.plot([uav_coords[u, 0], uav_coords[v, 0]],
                [uav_coords[u, 1], uav_coords[v, 1]],
                [uav_coords[u, 2], uav_coords[v, 2]], 'k--')
    ax.set_title(title)


with ThreadPoolExecutor(max_workers=2) as executor:
    greedy_future = executor.submit(greedy)
    shortest_future = executor.submit(shortest)
    # Wait for the threads to complete and get the results
    greedy_path = greedy_future.result()
    shortest_path = shortest_future.result()
    plot_path(greedy_path, "heuristic_path")
    plot_path(shortest_path, "dijkstra_based_path")

plt.show()

# num_runs = 100
# filename = 'output.csv'

# with open(filename, 'w', newline='') as csvfile:
#     csv_writer = csv.writer(csvfile)
#     csv_writer.writerow(['Algorithm', 'Execution Time'])
#     for _ in range(num_runs):
#         results = {}
#         with ThreadPoolExecutor(max_workers=2) as executor:
#             greedy_future = executor.submit(greedy)
#             shortest_future = executor.submit(shortest)
#             # Wait for the threads to complete and get the results
#             results['greedy'] = greedy_future.result()
#             results['shortest'] = shortest_future.result()
#         for algorithm, execution_time in results.items():
#             csv_writer.writerow([algorithm, execution_time])