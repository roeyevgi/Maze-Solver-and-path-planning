import cv2
import numpy as np




class PathPlanner():
    def __init__(self):
        self.dfs = DFS()
        self.dijisktra = Dijisktra()
        self.a_star = A_Star()


    def cords_to_pts(self, cords):
      return [cord[::-1] for cord in cords]


    def find_path_and_display(self, graph, start, end, maze, method='DFS'):
        if method == 'DFS':
            paths = self.get_paths(graph, start, end)
            path_to_display = paths[0]

        elif method == 'DFS_Shortest':
            paths_N_costs = self.get_paths_cost(graph, start, end)
            paths = paths_N_costs[0]
            costs = paths_N_costs[1]
            min_cost = min(costs)
            path_to_display = paths[costs.index(min_cost)]

        elif method == 'dijisktra':
            if not self.dijisktra.shortest_path_found:
                print('Finding shortest route...')
                self.dijisktra.find_best_routes(graph, start, end)
            path_to_display = self.dijisktra.shortest_path

        elif method == 'a_star':
            if not self.a_star.shortest_path_found:
                print('Finding shortest route...')
                self.a_star.find_best_routes(graph, start, end)
            path_to_display = self.a_star.shortest_path
        
        print(f'path_to_display: {path_to_display}')
        path_pts_to_display = self.cords_to_pts(path_to_display)
        self.draw_path_on_maze(maze, path_pts_to_display, method)
        cv2.waitKey(0)

    
    def draw_path_on_maze(self,maze,shortest_path_pts,method):
        
        maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
        self.choosen_route = np.zeros_like(maze_bgr)

        rang = list(range(0,254,25))
        
        depth = maze.shape[0]
        for i in range(len(shortest_path_pts)-1):
            per_depth = (shortest_path_pts[i][1])/depth

            # Blue : []   [0 1 2 3 251 255 251 3 2 1 0] 0-depthperc-0
            # Green :[]     depthperc
            # Red :  [] 100-depthperc
            color = ( 
                      int(255 * (abs(per_depth+(-1*(per_depth>0.5)))*2) ),
                      int(255 * per_depth),
                      int(255 * (1-per_depth))
                    )
            cv2.line(maze_bgr,shortest_path_pts[i] , shortest_path_pts[i+1], color)
            cv2.line(self.choosen_route,shortest_path_pts[i] , shortest_path_pts[i+1], color,3)

        img_str = 'maze (Found Path) [' + method +']'
        cv2.namedWindow(img_str,cv2.WINDOW_FREERATIO)
        cv2.imshow(img_str, maze_bgr)           
        self.img_shortest_path = maze_bgr.copy()


# DFS: Depth-first search
class DFS():
    def __init__(self):
        pass

    # Recursive function.
    def get_paths(self, graph, start, end, path=[]):
        path = path + [start]

        # Base conditions:
        if start == end:
            return [path]
        # Boundary case:
        if start not in graph.keys():
            return []
        
        # Store all possible paths from start to end.
        paths = []

        for node in graph[start].keys():
            if node not in paths and node != "case":
                new_paths = self.get_paths(graph, node, end, path)
                for path in new_paths:
                    paths.append(path)
        
        return paths
    
    
    def get_paths_cost(self, graph, start, end, path=[], cost=0, trav_cost=0):
        path = path + [start]
        cost = cost + trav_cost

        # Base conditions:
        if start == end:
            return [path], [cost]
        # Boundary case:
        if start not in graph.keys():
            return [], 0
        
        # Store all possible paths from start to end.
        paths = []
        # Store the cost of each possible path.
        costs = []

        for node in graph[start].keys():
            if node not in path and node != "case":
                new_paths, new_costs = self.get_paths_cost(graph, node, end, path, cost, graph[start][node]['cost'])
                for path in new_paths:
                    paths.append(path)
                for cost in new_costs:
                    costs.append(cost)
        
        return paths, costs
    


class Heap():
    def __init__(self):
        self.array = []
        self.size = 0
        self.pos_of_vertices = []


    def new_minHeap_node(self, vertex, dist):
        return ([vertex, dist])
    

    def swap_nodes(self, a, b):
        temp = self.array[a]
        self.array[a] = self.array[b]
        self.array[b] = temp
    

    def minHeapify(self, node_idx):
        smallest_node = node_idx

        left = node_idx * 2 + 1
        right = node_idx * 2 + 2

        if left < self.size and self.array[left][1] < self.array[smallest_node][1]:
            smallest_node = left
        if right < self.size and self.array[right][1] < self.array[smallest_node][1]:
            smallest_node = right

        if smallest_node != node_idx:
            self.pos_of_vertices[self.array[node_idx][0]] = smallest_node
            self.pos_of_vertices[self.array[smallest_node][0]] = node_idx
            self.swap_nodes(node_idx, smallest_node)
            self.minHeapify(smallest_node)
    
    # Extract the root. (smallest node)
    def extract_min(self):
        # If the priority queue is empty.
        if self.size == 0:
            return 
        root = self.array[0]
        last_node = self.array[self.size-1]
        self.array[0] = last_node
        self.pos_of_vertices[root[0]] = self.size - 1
        self.pos_of_vertices[last_node[0]] = 0
        self.size -= 1
        # Reorgenize the heap.
        self.minHeapify(0)
        return root
    

    def decrease_key(self, vertex, dist):
        vertex_idx = self.pos_of_vertices[vertex]
        self.array[vertex_idx][1] = dist
        # // is a Floor division.
        while vertex_idx > 0 and self.array[vertex_idx][1] < self.array[(vertex_idx-1)//2][1]:
            # Update the position of the current node and its parent.
            self.pos_of_vertices[self.array[vertex_idx][0]] = (vertex_idx - 1) / 2
            self.pos_of_vertices[self.array[(vertex_idx-1)//2][0]] = vertex_idx
            # Swar the current node with its parent.
            self.swap_nodes(vertex_idx, (vertex_idx-1)//2)
            # Navigate to parent and start process again.
            vertex_idx = (vertex_idx-1)//2

    # Check if a vertex is in min heap or not.
    def is_in_min_heap(self, vertex):
        if self.pos_of_vertices[vertex] < self.size:
            return True
        return False
    


class Dijisktra():
    def __init__(self):
        self.shortest_path_found = False
        self.shortest_path = []
        self.minHeap = Heap()
        self.dijisktra_node_visited = 0

        # Creating dictionaries to save the relationships between vertexs and indices.
        # The minHeap is only takes integers.
        self.idxs2vrtxs = {}
        self.vtrxs2idxs = {}

    
    def find_best_routes(self, graph, start, end):
        # Because the minHeap is only takes integers.
        start_idx = [idx for idx, key in enumerate(graph.items()) if key[0] == start][0]

        # Distance list to store the distance from each node.
        dist = []
        # Store the shortest subpaths. (parent_idx = closest_child_idx)
        parent = []

        # Setting the size of the min heap to be the number of keys in the graph.
        self.minHeap.size = len(graph.keys())

        for idx, vertex in enumerate(graph.keys()):
            # Initialize the dist for all vertices as inf.
            dist.append(1e7)
            self.minHeap.array.append(self.minHeap.new_minHeap_node(idx, dist[idx]))
            self.minHeap.pos_of_vertices.append(idx)

            # Initialize the parent with -1 for all indices.
            parent.append(-1)
            # Update the dictionaries of vertices and their indices.
            self.vtrxs2idxs[vertex] = idx
            self.idxs2vrtxs[idx] = vertex
        
        dist[start_idx] = 0
        self.minHeap.decrease_key(start_idx, dist[start_idx])

        while self.minHeap.size != 0:
            self.dijisktra_node_visited += 1
            current_top = self.minHeap.extract_min()
            # u is the current node.
            u_idx = current_top[0]
            u = self.idxs2vrtxs[u_idx]
            for v in graph[u]:
                if v != 'case':
                    v_idx = self.vtrxs2idxs[v]
                    # Check if found the shortest distance to v + new found distanse < known dist
                    if self.minHeap.is_in_min_heap(v_idx) and dist[u_idx] != 1e7 and graph[u][v]['cost'] + dist[u_idx] < dist[v_idx]:
                        # Update the dist for neighbor node.
                        dist[v_idx] = graph[u][v]['cost'] + dist[u_idx]
                        self.minHeap.decrease_key(v_idx, dist[v_idx])

                        parent[v_idx] = u_idx
            # If the end is already been visited. (shortest path already been found)
            if u == end:
                break
        shortest_path = []
        self.return_shortest_route(parent, start_idx, self.vtrxs2idxs[end], shortest_path)
        # Return route (reversed) from start to end.
        self.shortest_path = shortest_path[::-1]
        self.shortest_path_found = True
    
    def return_shortest_route(self, parent, start, end, route):
        # Keep updating the shortest route from end to start by visiting closest vertices.
        route.append(self.idxs2vrtxs[end])
        # If we reached the start position, stop.
        if end == start:
            return
        # Visit closest neighbor.
        end = parent[end]
        # Recursive call with new end.
        self.return_shortest_route(parent, start, end, route)
        

class A_Star(Dijisktra):
    def __init__(self):
        # Use the init of Dijisktra class.
        super().__init__()
        # Count how many node were visited to reach the goal.
        self.a_star_node_visited = 0

    
    def euclidean_distance(self, a, b):
        return np.sqrt(pow((a[0]-b[0]), 2) + pow((a[1]-b[1]), 2))
    
    # Function overriding.
    def find_best_routes(self, graph, start, end):
        # Because the minHeap is only takes integers.
        start_idx = [idx for idx, key in enumerate(graph.items()) if key[0] == start][0]

        # The cost of reaching every node from the start.
        cost_to_node = []
        # Distance list to store the distance from each node.
        dist = []
        # Store the shortest subpaths. (parent_idx = closest_child_idx)
        parent = []

        # Setting the size of the min heap to be the number of keys in the graph.
        self.minHeap.size = len(graph.keys())

        for idx, vertex in enumerate(graph.keys()):
            # Initialize the list for all vertices as inf.
            cost_to_node.append(1e7)
            # Initialize the dist for all vertices as inf.
            dist.append(1e7)
            self.minHeap.array.append(self.minHeap.new_minHeap_node(idx, dist[idx]))
            self.minHeap.pos_of_vertices.append(idx)

            # Initialize the parent with -1 for all indices.
            parent.append(-1)
            # Update the dictionaries of vertices and their indices.
            self.vtrxs2idxs[vertex] = idx
            self.idxs2vrtxs[idx] = vertex

        cost_to_node[start_idx] = 0
        dist[start_idx] = cost_to_node[start_idx] + self.euclidean_distance(start, end)
        self.minHeap.decrease_key(start_idx, dist[start_idx])

        while self.minHeap.size != 0:
            self.a_star_node_visited += 1
            current_top = self.minHeap.extract_min()
            # u is the current node.
            u_idx = current_top[0]
            u = self.idxs2vrtxs[u_idx]

            for v in graph[u]:
                if v != 'case':
                    v_idx = self.vtrxs2idxs[v]
                    # Check if found the shortest distance to v + new found distanse < known dist
                    if self.minHeap.is_in_min_heap(v_idx) and dist[u_idx] != 1e7 and graph[u][v]['cost'] + cost_to_node[u_idx] < cost_to_node[v_idx]:
                        # Update the dist for neighbor node.
                        cost_to_node[v_idx] = graph[u][v]['cost'] + cost_to_node[u_idx]
                        dist[v_idx] = cost_to_node[v_idx] + self.euclidean_distance(v, end)
                        self.minHeap.decrease_key(v_idx, dist[v_idx])

                        parent[v_idx] = u_idx
            # If the end is already been visited. (shortest path already been found)
            if u == end:
                break
        shortest_path = []
        self.return_shortest_route(parent, start_idx, self.vtrxs2idxs[end], shortest_path)
        # Return route (reversed) from start to end.
        self.shortest_path = shortest_path[::-1]
        self.shortest_path_found = True

