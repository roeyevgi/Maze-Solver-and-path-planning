import cv2
import numpy as np

draw_interest_points = True

class Graph():
    def __init__(self):
        self.graph = {}

        self.start = 0
        self.end = 0

    def add_vertex(self, vertex, neighbor=None, case=None, cost=None):
        # If the vertex is alreay present in the graph.
        if vertex in self.graph.keys():
            # Update the neighbor.
            self.graph[vertex][neighbor] = {}
            self.graph[vertex][neighbor]['case'] = case
            self.graph[vertex][neighbor]['cost'] = cost
        else:
            self.graph[vertex] = {}
            self.graph[vertex]['case'] = case
    
    def display_graph(self):
        for key, value in self.graph.items():
            print(f'Key {key} has the value {value} ')
            


class RobotMapper():
    def __init__(self):
        self.graph_is_ready = False
        # Crop control for removing the maze boundaries.
        self.crop_amount = 5  # Pixels.
        self.graph = Graph()
        # Display the connection between the nodes.
        self.maze_connect = []

        # Connection status to each vertex.
        self.connected_left = False
        self.connected_upleft = False
        self.connected_up = False
        self.connected_upright = False

    def display_connected_nodes(self, current_node, neighbor_node, case='Unkown', color=(0,0,255)):
        current_pixel = (current_node[1],current_node[0])
        neighbor_pixel = (neighbor_node[1], neighbor_node[0])
        print(f'-------------------) CONNECTED >> {case} <<')
        self.maze_connect = cv2.line(self.maze_connect, current_pixel, neighbor_pixel, color, 1)
        cv2.imshow('Node connected', self.maze_connect)

    # connect the current node to its neighbors in immediate [left -> top-right] region.
    def connect_neighbors(self, maze, node_row, node_col, case, step_l=1, step_up=0, tot_connected=0):
        current_node = (node_row, node_col)
        if maze[node_row-step_up][node_col-step_l] > 0:
            neighbor_node = (node_row-step_up, node_col-step_l)
            if neighbor_node in self.graph.graph.keys():
                neighbor_case = self.graph.graph[neighbor_node]['case']
                cost = max(abs(step_l), abs(step_up))
                tot_connected += 1

                self.graph.add_vertex(current_node,neighbor_node, neighbor_case, cost)
                self.graph.add_vertex(neighbor_node, current_node, case, cost)
                print(f'\nConnected {current_node} to {neighbor_node} with case [step_l,step_up] = [{step_l},{step_up}] & cost -> {cost}')

                if not self.connected_left:
                    # Vertex has connected to its left neighbor.
                    self.display_connected_nodes(current_node, neighbor_node, 'LEFT', (0,0,255))
                    self.connected_left = True
                    # Check up_left route next.
                    step_l = 1
                    step_up = 1
                    self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, tot_connected)

                if not self.connected_upleft:
                    # Vertex has connected to its up_left neighbor.
                    self.display_connected_nodes(current_node, neighbor_node, 'UPLEFT', (0,0,255))
                    self.connected_upleft = True
                    # Check up route next.
                    step_l = 0
                    step_up = 1
                    self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, tot_connected)

                if not self.connected_up:
                    # Vertex has connected to its up neighbor.
                    self.display_connected_nodes(current_node, neighbor_node, 'UP', (0,0,255))
                    self.connected_up = True
                    # Check up_right next.
                    step_l = -1
                    step_up = 1
                    self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, tot_connected)

                if not self.connected_upright:
                    # Vertex has connected to its up_right neighbor.
                    self.display_connected_nodes(current_node, neighbor_node, 'UPRIGHT', (0,0,255))
                    self.connected_upright = True
                # Cicle has been completed, all disired neighbors has been found and connected.

            # If the cicle has not been completed.
            if not self.connected_upright:
                # Look a little more to the left.
                if not self.connected_left:
                    step_l += 1
                # Look a little more to the up left diagonal.
                elif not self.connected_upleft:
                    step_l += 1
                    step_up += 1
                # Look a little more up.
                elif not self.connected_up:
                    step_up += 1
                # Look a little more to the up right diagonal.
                elif not self.connected_upright:
                    step_l -= 1
                    step_up += 1
                self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, tot_connected)
        # No path in the direction you are looking, Cycle to next direction
        else:
            # If there is a wall to the left, start looking up left.
            if not self.connected_left:
                self.connected_left = True
                step_l = 1
                step_up = 1
                self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, tot_connected)
            # If there is a wall up left, start looking up.
            elif not self.connected_upleft:
                self.connected_upleft = True
                step_l = 0
                step_up = 1
                self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, tot_connected)
            # If there is a wall above, start looking up right.
            elif not self.connected_up:
                self.connected_up = True
                step_l = -1
                step_up = 1
                self.connect_neighbors(maze, node_row, node_col, case, step_l, step_up, tot_connected)
            elif not self.connected_upright:
                self.connected_upright = True
                step_l = 0
                step_up = 0
                return 


    def maze_to_graph(self, extrected_maze):
        if not self.graph_is_ready:
            cv2.imshow('Extracted maze', extrected_maze)
            thinned = cv2.ximgproc.thinning(extrected_maze)
            cv2.imshow('Thinned maze', thinned)

            # Dilate and perform thinning again to minimize unnecceary interest points.
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2,2))
            thinned_dilated = cv2.morphologyEx(thinned, cv2.MORPH_DILATE, kernel)
            _, bw2 = cv2.threshold(thinned_dilated, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            thinned = cv2.ximgproc.thinning(bw2)
            cv2.imshow('Maze (thinned X2)', thinned)
            thinned_cropped = thinned[self.crop_amount:thinned.shape[0]-self.crop_amount,
                    self.crop_amount:thinned.shape[1]-self.crop_amount]
            cv2.imshow('Maze (thinned X2 + crop)', thinned_cropped)

            # Overlay fround path on maze occupency grid.
            extrected_maze_cropped = extrected_maze[self.crop_amount:extrected_maze.shape[0]-self.crop_amount,
                                                    self.crop_amount:extrected_maze.shape[1]-self.crop_amount]
            extrected_maze_cropped = cv2.cvtColor(extrected_maze_cropped, cv2.COLOR_GRAY2BGR)
            extrected_maze_cropped[thinned_cropped>0] = (0, 255, 255)
            cv2.imshow('Maze (thinned X2 + crop + overlay)', extrected_maze_cropped)

            self.one_pass(thinned_cropped)
            cv2.waitKey(0)


    def one_pass(self, maze):
        # Clear the graph every time one_pass is been called.
        self.graph.graph.clear()
        # Initialize maze_connect with colored maze.
        self.maze_connect = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
        cv2.namedWindow('Nodes connected', cv2.WINDOW_FREERATIO)
        
        # Counters for IP.
        turns = 0
        junc_3 = 0
        junc_4 = 0

        # Converting the color of the maze from gray scale to BGR.
        maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
        cv2.namedWindow("Maze (interest point)", cv2.WINDOW_FREERATIO)
        rows = maze.shape[0]
        cols = maze.shape[1]
        # Find all possible interest point like start and end point and turns or dead ends.
        for row in range(rows):
            for col in range(cols):
                if maze[row][col] == 255:
                    # if debug_mapping:
                    #     self.maze_connect = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
                    # Possible IP. (intrest point) ==> Find surrounding pixels intensities
                    top_left, top, top_right, right, btm_right, btm, btm_left, left, paths = self.get_surround_pixel_intensities(maze.copy(), row, col)
                    
                    if row == 0 or row == (rows-1) or col == 0 or col == (cols-1):
                        if row == 0:
                            # Start point.
                            maze_bgr[row][col] = (0, 128, 255) # Orange
                            cv2.imshow('Maze (IP)', maze_bgr)
                            # Add the start point to the graph
                            self.graph.add_vertex((row,col), case='_Start_')
                            self.graph.start = (row, col)
                        else:
                            # Maze exit
                            maze_bgr[row][col] = (0, 255, 0) # Green
                            cv2.imshow('Maze (IP)', maze_bgr)
                            # Add the start point to the graph
                            self.graph.add_vertex((row,col), case='_End_')
                            self.graph.end = (row, col)
                            self.reset_connect_parameters()
                            self.connect_neighbors(maze, row, col, case='_End_')
                    
                    # Check if it is a dead end.
                    elif paths == 1:
                        crop = maze[row-1:row+2, col-1:col-2]
                        maze_bgr[row][col] = (0, 0, 255) # Red
                        if draw_interest_points:
                            maze_bgr = cv2.circle(maze_bgr, (col, row), 10, (0,0,255), 2)
                            cv2.imshow('Maze (IP)', maze_bgr)
                            # Add the point to the graph
                            self.graph.add_vertex((row,col), case='_DeadEnd_')
                            self.graph.start = (row, col)
                            self.reset_connect_parameters()
                            self.connect_neighbors(maze, row, col, case='_DeadEnd_')

                    # Check if it is a Turn or a stright path.
                    elif paths == 2:
                        crop = maze[row-1:row+2, col-1:col+2]
                        nzero_loc = np.nonzero(crop > 0)
                        nzero_ptA = (nzero_loc[0][0], nzero_loc[1][0])
                        nzero_ptB = (nzero_loc[0][2], nzero_loc[1][2])
                        # If it is a turn.
                        if not (((2 - nzero_ptA[0]) == nzero_ptB[0]) and (2 - nzero_ptA[1] == nzero_ptB[1])):
                            maze_bgr[row][col] = (255, 0, 0) # Blue.
                            cv2.imshow('Maze (IP)', maze_bgr)
                            # Add the point to the graph
                            self.graph.add_vertex((row,col), case='_Turn_')
                            self.graph.start = (row, col)
                            self.reset_connect_parameters()
                            self.connect_neighbors(maze, row, col, case='_Turn_')
                            turns += 1

                    elif paths > 2:
                        # 3 paths junction.
                        if paths == 3:
                            maze_bgr[row][col] = (255, 244, 128)
                            if draw_interest_points:
                                maze_bgr = cv2.ellipse(maze_bgr, (col,row), (4,5),0 , 0, 360, (144,140,255), 2)
                            cv2.imshow('Maze (IP)', maze_bgr)
                            # Add the point to the graph
                            self.graph.add_vertex((row,col), case='_3-Junc_')
                            self.graph.start = (row, col)
                            self.reset_connect_parameters()
                            self.connect_neighbors(maze, row, col, case='_3-Junc_')
                            junc_3 += 1
                        else:
                            maze_bgr[row][col] = (128, 0, 128)
                            if draw_interest_points:
                                maze_bgr = cv2.rectangle(maze_bgr, (col-10, row-10), (col+10, row+10), (255,140,144), 2)
                            cv2.imshow('Maze (IP)', maze_bgr)
                            # Add the point to the graph
                            self.graph.add_vertex((row,col), case='_4-Junc_')
                            self.graph.start = (row, col)
                            self.reset_connect_parameters()
                            self.connect_neighbors(maze, row, col, case='_4-Junc_')
                            junc_4 += 1
        print(f'\nInterest points: \n[Turns, 3-junction, 4-Jonction] = [{turns}, {junc_3}, {junc_4}]')
        
    # Returns the surrounding pixels for a current pixel and how many paths it has.
    def get_surround_pixel_intensities(self, maze, current_row, current_col):
        # Convert every value greater then 1 (threshold) to 1 (max value).
        maze = cv2.threshold(maze, 1, 1, cv2.THRESH_BINARY)[1]
        rows = maze.shape[0]
        cols = maze.shape[1]

        # If the point is at boundary condition
        top_row = False
        btm_row = False
        l_col = False
        r_col = False

        if current_row == 0:
            top_row = True
        if current_row == (rows-1) :
            btm_row = True
        if current_col == 0:
            l_col = True
        if current_col == (cols-1):
            r_col == True
        
        if top_row or l_col:
            top_left = 0
        else:
            top_left = maze[current_row-1][current_col-1]

        if top_row or r_col:
            top_right = 0
        else:
            top_right = maze[current_row-1][current_col+1]

        if btm_row or l_col:
            btm_left = 0
        else:
            btm_left = maze[current_row+1][current_col-1]

        if btm_row or r_col:
            btm_right = 0
        else:
            btm_right = maze[current_row+1][current_col+1]

        if top_row:
            top = 0
        else:
            top = maze[current_row-1][current_col]

        if btm_row:
            btm = 0
        else:
            btm = maze[current_row+1][current_col]
        if l_col:
            left = 0
        else:
            left = maze[current_row][current_col-1]
        if r_col:
            right = 0
        else:
            right = maze[current_row][current_col+1]
        # Sum how many path the pixel has.
        num_of_pathways = ( top_left + top + top_right +
                            left     +  0  + right     +
                            btm_left + btm + btm_right
                          )
        
        return top_left, top, top_right, right, btm_right, btm, btm_left, left, num_of_pathways

    # Reset parameters of each vertex connection
    def reset_connect_parameters(self):
        self.connected_left = False
        self.connected_upleft = False
        self.connected_up = False
        self.connected_upright = False
