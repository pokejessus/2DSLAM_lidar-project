import math
import numpy as np
from env_lidar import Fakelidar, check_collision_test
from collections import deque

class path_finding:
    def __init__(self, start, goal, MapDimensions, robot_radius, sensor: Fakelidar):
        # variables to run the algorithm
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.spotted_goal = False
        self.robot_radius = robot_radius
        self.current_node = 0 

        self.mapH, self.mapW = MapDimensions

        # Represent graph
        self.nodes = [start]
        self.edge = []

        # map for obstacle detection
        self.sensor = sensor

        # # Obstacles
        # self.obstacles = []                 # obstacle ids
        # self.obsDim = []                    # dimensions of obstacles
        # self.obsNb = len(self.obstacles)    # nb of obstacles

        # Path
        self.goalstate = None
        self.path = []


    def getCoords(self, n) -> tuple:
        return self.nodes[n]
       
    def add_node(self, x, y, visited = False, clockwise = False):
        '''
        x, y: coords
        '''
        self.nodes.append((x, y, visited, clockwise))
    
    def remove_node(self, n):
        self.nodes.pop(n)
        edge_to_delete = []

        # selecting all the edges we want to delete
        for i in range(len(self.edge)):
            if (n in self.edge[i]):
                edge_to_delete.append(i)

            # updating the new indexes
            if (self.edge[i][0] > n):
                self.edge[i] = (self.edge[i][0] - 1, self.edge[i][1])
            
            if (self.edge[i][1] > n):
                self.edge[i] = (self.edge[i][0], self.edge[i][1] - 1)

        for i in range(len(edge_to_delete)):
            self.edge.pop(edge_to_delete[i] - i)


    def add_edge(self, n1, n2):
        # add edge between nods index1 and node index2
        self.edge.append((n1, n2))

    def remove_edge(self, edge):
        self.edge.pop(edge) # index of edge

    def number_of_nodes(self):
        return len(self.nodes)
    
    def find_parent(self, n1_index):
        if (n1_index == 0):
            return 0
        for edge in self.edge:
            if edge[1] == n1_index:
                return edge[0]
            

    # BFS for single connected component
    def bfs(self, idx_parent_node):

        q = deque()

        # Get the list of child
        for e in self.edge:
            if (e[0] == idx_parent_node):
                q.append(e[1])

        while q:
            curr = q.popleft()

            if (self.nodes[curr][2] == False):
                return curr

            # visit all the unvisited
            # neighbours of current node
            for e in self.edge:
                if (e[0] == curr):
                    q.append(e[1])

        return None


    
    # Measure distance between two nodes that are adjacent
    def distance(self, n1, n2):
        x1, y1 = self.nodes[n1]
        x2, y2 = self.nodes[n2]
        px = (float(x2) - float(x1))**2 
        py = (float(y2) - float(y1))**2
        return math.sqrt(px + py)
    
    def angle(self, n1, n2):
        x_1, y_1, _, _ = self.nodes[n1]
        x_2, y_2, _, _ = self.nodes[n2]
        angle = np.arctan((y_2-y_1)/(x_2-x_1))

        return angle

    
    # Euclidean distance between two lidar points
    def euclidean_distance(self, p1, p2):
        return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)
    
    def cluster(self, ls_point, delta: float = 0.5):
        ls_cluster = []
        in_cluster = False

        for p1 in ls_point:
            in_cluster = False
            # Initialize the list

            for cluster in ls_cluster:
                for p2 in cluster:
                    if(self.euclidean_distance(p1, p2) < delta and not in_cluster):
                        cluster.append(p2)
                        in_cluster = True


            if (not in_cluster):
                ls_cluster.append([p1])

        return ls_cluster
    
    def find_extremeties(self, cluster):
        ls_angle = []

        # extract the angle
        for pt in cluster:
            ls_angle.append(pt.angle)

        done = True

        # Rotate the angle until you get no more 0 and 360 angle
        for angle in ls_angle:
            if (angle > 359 and angle < 1):
                done = False
                break
            
        while not done:
            for i in range(len(ls_angle)):
                ls_angle[i] = (ls_angle[i] + 1)%360

            done = True

            for angle in ls_angle:
                if (angle > 359 and angle < 1):
                    done = False

        i = np.argmin(ls_angle)
        j = np.argmax(ls_angle)

        return cluster[i], cluster[j]


    def find_new_node(self, ls_cluster):
        ls_extremeties = []
        clockwise = True

        # Create a list of extremeties
        for cluster in ls_cluster:
            ls_extremeties.append(self.find_extremeties(cluster))

        ls_result_idx = []

        # Find all the half points
        for i in range(len(ls_extremeties)):
            p1 = ls_extremeties[i][1]
            p2 = (ls_extremeties[i + 1]%len(ls_extremeties))[0]

            # Find the point furthest away from the robot
            if (p1.distance > p2.distance):
                max_point = p1
                min_pt = p2
                clockwise = False

            else:
                max_point = p2
                min_pt = p1
                clockwise = True


            delta_x = max_point.x - min_pt.x
            delta_y = max_point.y - min_pt.y

            self.add_node(min_pt.x + delta_x/2, min_pt.y + delta_y/2, visited= False, clockwise=clockwise)
            self.add_edge(self.current_node, len(self.nodes)-1)
            ls_result_idx.append((len(self.nodes)-1, clockwise))

        return np.random.choice(ls_result_idx)


        
    def go_to(self, idx_node, clockwise):

        ls_pts_path = [] # List of points on the path

        if clockwise:
            new_point_x = self.nodes[self.current_node][0]
            new_point_y = self.nodes[idx_node[0]][1]

        else:
            new_point_x = self.nodes[self.current_node][0]
            new_point_y = self.nodes[idx_node[0]][1]

        self.add_node(new_point_x, new_point_y, False, clockwise=None)
        self.add_edge(self.current_node, len(self.nodes)-1)
        self.add_edge(len(self.nodes)-1, idx_node) 

        distance = self.distance(self.current_node, -1)
        angle = self.angle(self.current_node, -1)

        ls_pts_path.append((new_point_x, new_point_y, distance, angle))

        for i in range(len(self.edge)):
            if (self.edge[i] == (self.current_node, idx_node)):
                self.remove_edge(i)
                break

        # Move the robot to new_point

        # robot moves to (new_point_x, new_point_y)
        # robot moves to the self.nodes[idx_node] 

        self.current_node = -1

        distance = self.distance(self.current_node, idx_node)
        angle = self.angle(self.current_node, idx_node)
        ls_pts_path.append((self.getCoords(idx_node)[0], self.getCoords(idx_node)[1], distance, angle))

        self.current_node = idx_node
        self.nodes[self.current_node] = (self.nodes[self.current_node][0], self.nodes[self.current_node][1], True, None)

    
        return ls_pts_path # list of (x, y, distance, angle)


    def detect_dead_end(self, ls_cluster):
        if (len(ls_cluster) != 1):
            return False
        
        ls_angle = []

        cluster = ls_cluster[0]
        
        distance = 0
        index = 0

        for i in range(len(cluster)):
            if (abs(cluster[i].distance - cluster[(i + 1)%len(cluster)].distance) > distance):
                index = i
                distance = abs(cluster[i].distance - cluster[(i + 1)%len(cluster)].distance)

        parent_node_index = self.find_parent(self.find_parent(self.current_node))
        x_p, y_p, _, _ = self.nodes[parent_node_index]
        x, y, _, _ = self.nodes[self.current_node]

        angle = np.arctan((y_p-y)/(x_p-x))
        
        uncertainty = 0.2
        angle_deg = angle*(180/np.pi)

        if (angle_deg - uncertainty < cluster[index].angle < angle_deg + uncertainty):
            return True
        
        return False
    
    # Check if reached goal

    def list_child(self, index):
        
        ls_child = []

        for e in self.edge:
            if (e[0] == index):
                if (not self.nodes[e[1]][2]):
                    ls_child.append(e[1])

        return ls_child
    
    
    def go_back(self):

        ls_path = []

        done = False

        while (not done):

            if (self.current_node == 0):
                raise Exception("All nodes have been explored")
            
            # If have intermediate node
            if (self.nodes[self.current_node][3] is not None):

                parent_node_index = self.find_parent(self.find_parent(self.current_node))
                intermediate_index = self.find_parent(self.current_node)

                distance = self.distance(self.current_node, intermediate_index)
                angle = self.angle(self.current_node, intermediate_index)
                ls_path.append(self.getCoords(intermediate_index)[0], self.getCoords(intermediate_index)[1], distance, angle)

                distance = self.distance(intermediate_index, parent_node_index)
                angle = self.angle(intermediate_index, parent_node_index)
                ls_path.append(self.getCoords(parent_node_index)[0], self.getCoords(parent_node_index)[1], distance, angle)
                
                self.current_node = parent_node_index

            # If not an intermediate, go directly
            else:
                intermediate_index = self.find_parent(self.current_node)

                distance = self.distance(self.current_node, intermediate_index)
                angle = self.angle(self.current_node, intermediate_index)
                ls_path.append(self.getCoords(intermediate_index)[0], self.getCoords(intermediate_index)[1], distance, angle)
                
                self.current_node = intermediate_index

            # What if the children of the previous node has already been explored
            idx_node = self.bfs(self.current_node)

            if (idx_node is not None):
                done = True

        # Getting path between current node and idx_node (which is not visited)
        temp = idx_node
        ls_nodes = []

        while (self.current_node != temp):
            temp = self.find_parent(temp)
            ls_nodes.append(temp)

        ls_nodes = ls_nodes[:-1]


        # Looking at the list from right to left
        for i in range(len(ls_nodes)-1, -1, -1):
            distance = self.distance(self.current_node, ls_nodes[i])
            angle = self.angle(self.current_node, ls_nodes[i])
            ls_path.append(self.getCoords(ls_nodes[i])[0], self.getCoords(ls_nodes[i])[1], distance, angle)

            self.current_node = ls_nodes[i]


        ls_path += self.go_to(idx_node, self.nodes[idx_node][3]) # concatenate both list

        return ls_path


    def run(self):
        x, y, _, _ = self.nodes[self.current_node]
        ls_point = self.sensor.scan((x, y))

        ls_cluster = self.cluster(ls_point)

        # What if ls_cluster is empty?
        if (len(ls_cluster) == 0):
            self.add_node(self.getCoords(self.current_node)[0], self.getCoords(self.current_node)[1] + 600, visited=True, clockwise=None)
            distance = self.distance(self.current_node, len(self.nodes)-1)
            angle = self.angle(self.current_node, len(self.nodes)-1)

            self.current_node = len(self.nodes)-1

            return [(self.getCoords(self.current_node)[0], self.getCoords(self.current_node)[1],distance, angle)]
        
        if (self.detect_dead_end(ls_cluster)):
            return self.go_back()
            
        else:
            idx = self.find_new_node(ls_cluster) 
            return self.go_to(*idx)
            



                
                


        





                        
    
