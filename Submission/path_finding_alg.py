import math
import numpy as np
from env_lidar import Fakelidar, display_points, display_clusters_scatter
from collections import deque


"""
TODO:
    - replace scan by using the global map inside the go_back function
"""


class path_finding:
    def __init__(self, start, goal, MapDimensions, robot_radius, sensor: Fakelidar, verbose: bool = False):
        # variables to run the algorithm
        self.start = (start[0], start[1], True, None)
        self.goal = goal
        self.goalFlag = False
        self.spotted_goal = False
        self.robot_radius = robot_radius
        self.current_node = 0 

        self.mapH, self.mapW = MapDimensions
        # Path and Map Memory
        self.global_map_points = [] # Stores all observed obstacle coordinates
        self.traversed_path = [self.start[:2]] # Stores the history of (x,y) positions

        # Represent graph
        self.nodes = [self.start]
        self.edge = []

        # map for obstacle detection
        self.sensor = sensor

        self.verbose = verbose

        # Path
        self.goalstate = None
        self.path = []

    def getCoords(self, n) -> tuple:
        """
        input: index
        output: the coordinate of the node
        """
        return self.nodes[n]
       
    def add_node(self, x, y, visited = False, intermediate = False):
        """
        input: x, y (the coordinate), if it got visited, and if it is an intermediate node
        ouput: None

        Adds the node to the list of node available
        """
        self.nodes.append((x, y, visited, intermediate))
    
    def remove_node(self, n):
        """
        input: index of the node you want to remove
        output: None

        Removes the node in the list + the edges adjacent to it
        """
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
        """
        input: two index nodes
        output: None

        Adds an edge between two index nodes
        """
        # add edge between nods index1 and node index2
        self.edge.append((n1, n2))

    def remove_edge(self, edge_idx):
        """
        input: edge index
        output: None

        Removes the edge at that index
        """
        self.edge.pop(edge_idx) # index of edge
    
    def remove_edge_between(self, n1, n2):
        """
        input: two index nodes
        ouput: None

        Removes the edge between the two index node from n1 to n2
        """
        ls_remove = []
        for i in range(len(self.edge)):
            if self.edge[i] == (n1, n2):
                ls_remove.append(i)
        
        for i, idx in enumerate(ls_remove):
            self.edge.pop(idx - i)

    def number_of_nodes(self):
        """
        input: None
        output: the number of nodes

        Calculates the number of nodes in the list of nodes
        """
        return len(self.nodes)
    
    def find_parent(self, n1_index):
        """
        input: index of the node
        output: returns the parent index

        Find the parent of the node at the index 
        """
        if (n1_index == 0):
            return 0
        for edge in self.edge:
            if edge[1] == n1_index:
                return edge[0]
            
    def bfs(self, idx_parent_node):
        """
        input: index parent node
        output: return the index of the unvisted node

        Does a BFS search for a non-visited node
        """

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

    def distance(self, n1, n2):
        """
        input: two index node
        output: the Euclidean distance between them 
        
        Calculates the Euclidean distance between the two nodes 
        """
        x1, y1, _, _ = self.nodes[n1]
        x2, y2, _, _ = self.nodes[n2]
        px = (float(x2) - float(x1))**2 
        py = (float(y2) - float(y1))**2
        return math.sqrt(px + py)
    
    def angle(self, n1, n2):
        """
        input: two index nodes
        output: Returns the clockwise angle (in radiant) from point A to point B.

        0° points right (+x), and angles increase clockwise.
        Assumes an anti standard Cartesian coordinate system (y increases downward).
        """
        x_1, y_1, _, _ = self.nodes[n1]
        x_2, y_2, _, _ = self.nodes[n2]

        dx = x_2 - x_1
        dy = y_2 - y_1
        
        if dx == 0 and dy == 0:
            raise ValueError("Points A and B are identical. Direction is undefined.")
            
        # In image coordinates (y-down), atan2(dy, dx) naturally returns 
        # angles that increase clockwise from the +x axis.
        angle_rad = math.atan2(dy, dx)
        
        return angle_rad # math.degrees(angle_rad) % 360

    def euclidean_distance(self, p1, p2):
        """
        input: two LidarPoint
        ouput: the distance between the two lidar points

        Calculates the Euclidean distance between the two LidarPoints in cm.
        """
        return np.sqrt((p1.x / 10 - p2.x / 10)**2 + (p1.y / 10 - p2.y / 10)**2)

    def cluster(self, ls_point, margin_factor: float = 1.0, noise_margin: float = 5.0, min_cluster_size: int = 10):
        """
        input: list of lidarpoints, a margin factor, a noise margie and the minimum size of clusters
        output: all the lists of clusters (points that are close)

        Groups points sequentially using an adaptive threshold based on Lidar geometry.
        """
        if not ls_point:
            return[]

        ls_cluster = [[ls_point[0]]]

        for p in ls_point[1:]:
            last_cluster = ls_cluster[-1]
            last_point = last_cluster[-1]

            # 1. Calculate the relative angle difference (handling 360 wrap-around)
            diff_angle = (p.angle - last_point.angle) % 360

            # 2. Calculate the expected spatial gap caused by the Lidar's angle step.
            # Arc Length Formula: Arc = Radius * Angle(in radians)
            # We use the shorter distance of the two points to be conservative.
            min_dist = min(p.distance, last_point.distance)
            expected_gap = min_dist * math.radians(diff_angle)

            # 3. Compute the dynamic threshold for these specific two points
            adaptive_delta = (expected_gap * margin_factor) + noise_margin

            # 4. Compare the actual Euclidean distance to our dynamic threshold
            if self.euclidean_distance(p, last_point) < adaptive_delta:
                last_cluster.append(p)
            else:
                # The gap is larger than it mathematically should be -> New Obstacle
                ls_cluster.append([p])

        # -------------------------------------------------------------
        # 360-Degree Wrap-around Check (Merge last and first clusters)
        # -------------------------------------------------------------
        if len(ls_cluster) > 1:
            first_point = ls_cluster[0][0]
            last_point = ls_cluster[-1][-1]
            
            diff_angle = (first_point.angle - last_point.angle) % 360
            
            min_dist = min(first_point.distance, last_point.distance)
            expected_gap = min_dist * math.radians(diff_angle)
            adaptive_delta = (expected_gap * margin_factor) + noise_margin
            
            if self.euclidean_distance(first_point, last_point) < adaptive_delta:
                # Merge last cluster into the first one
                ls_cluster[0] = ls_cluster[-1] + ls_cluster[0]
                ls_cluster.pop()

        # Remove the clusters that are too small
        ls_remove = []
        for i, c in enumerate(ls_cluster):
            if len(c) < min_cluster_size:
                ls_remove.append(i)

        for i, idx in enumerate(ls_remove):
            ls_cluster.pop(idx - i)

        return ls_cluster

    def find_extremeties(self, cluster):
        """
        input: a cluster
        output: the two extremeties in that cluster

        This function finds the two extreme point (left most and right most points) in a cluster. 
        """
        ls_angle = []

        # extract the angle
        for pt in cluster:
            ls_angle.append(pt.angle)

        done = True

        # Rotate the angle until you get no more 0 and 360 angle
        for angle in ls_angle:
            if (angle > 359 or angle < 1):
                # print("THERE IS A CLUSTER OVERLAPPING 0")
                done = False
                break
            
        while not done:
            for i in range(len(ls_angle)):
                ls_angle[i] = (ls_angle[i] + 1)%360

            done = True

            for angle in ls_angle:
                if (angle > 359 or angle < 1):
                    done = False

        i = np.argmin(ls_angle)
        j = np.argmax(ls_angle)

        return cluster[i], cluster[j]
    
    def find_new_node(self, ls_cluster):
        """
        input: takes a list of clusters (which are lists of LidarPoint)
        output: returns the new node it found by using extremeties + safety margin

        Finds all the nodes between two extremeties of clusters (that it can see). 
        Important to note that it does not return the node that has an angle too close to a previously visited node.
        The same for the distance between them if too close. 
        """
        ls_extremeties = []
        for idx, cluster in enumerate(ls_cluster):
            ls_extremeties.append(self.find_extremeties(cluster))
            if self.verbose:
                print(f"The extremities of the cluster {idx} are: {ls_extremeties[-1]}")


        ls_result_idx = []
        
        # Calculate angle (in radiant) to parent once to avoid going backward
        parent_angle = None
        if self.current_node != 0:
            parent_node_index = self.find_parent(self.current_node)
            
            # Find the first non-intermediate parent to get a true directional vector
            while parent_node_index is not None and self.nodes[parent_node_index][3]:
                next_p = self.find_parent(parent_node_index)
                
                if next_p == parent_node_index or next_p is None:
                    break

                parent_node_index = next_p
                
            if parent_node_index is not None and parent_node_index != self.current_node:
                parent_angle = self.angle(self.current_node, parent_node_index)


        # Find all the half points
        for i in range(len(ls_extremeties)):
            p1 = ls_extremeties[i][1]
            p2 = (ls_extremeties[(i + 1)%len(ls_extremeties)])[0]

            # Get the point that is the furthest away from the robot
            if (p1.distance > p2.distance):
                max_point = p1
                min_pt = p2
            else:
                max_point = p2
                min_pt = p1

            # vector from the closest point to the furthest point
            delta_x = max_point.x - min_pt.x
            delta_y = max_point.y - min_pt.y

            # Global coordinates of the point between the 2 extremities
            x = self.nodes[self.current_node][0] + (min_pt.x + delta_x/2) / 10
            y = self.nodes[self.current_node][1] + (min_pt.y + delta_y/2) / 10

            # --- NEW PROXIMITY CHECK CODE ---
            is_duplicate = False
            duplicate_threshold_node = self.robot_radius * 3.5  
            duplicate_threshold_path = self.robot_radius * 2.5  
            
            # Check against existing nodes
            ls_remove = []
            for idx, node in enumerate(self.nodes):
                dist = math.hypot(x - node[0], y - node[1])
                if dist < duplicate_threshold_node:
                    if not node[2]:
                        ls_remove.append(idx)
                    
                    else:
                        is_duplicate = True
                        break
            
            # Remove the node that are not explored yet and that are too close to the new node
            for idx, rem in enumerate(ls_remove):
                if self.verbose:
                    print(f"\nNode to remover: {self.nodes[rem-idx]} at index {rem-idx}\n")
                
                self.remove_node(rem - idx)
                if self.current_node >= rem - idx:
                    self.current_node -= 1

                for j in range(len(ls_result_idx)):
                    if ls_result_idx[j] >= rem - idx:
                        ls_result_idx[j] -= 1
            
            # Check against traversed path (to avoid "seeing gaps" back into explored territory)
            if not is_duplicate:
                for px, py in self.traversed_path[::5]:
                    if math.hypot(x - px, y - py) < duplicate_threshold_path:
                        is_duplicate = True
                        break
                        
            # --- CHECK ANGLE TO PARENT (Don't go backwards at dead-ends) ---
            if not is_duplicate and parent_angle is not None:
                dx = x - self.nodes[self.current_node][0]
                dy = y - self.nodes[self.current_node][1]
                proposed_angle = math.atan2(dy, dx)
                
                # compute the difference of angle between the proposed angle and the parent angle
                diff = abs(math.degrees(proposed_angle - parent_angle))
                #diff = abs(proposed_angle - parent_angle)
                #diff = min(diff, 2 * math.pi - diff)
                
                if diff < 20: 
                    is_duplicate = True
            
            if not is_duplicate:
                self.add_node(x, y, False, False)
                new_idx = len(self.nodes) - 1
                self.add_edge(self.current_node, new_idx)
                ls_result_idx.append(new_idx)


        return ls_result_idx

    def find_door(self, ls_points):
        """
        input: list of LidarPoints
        output: A boolean for weither the robot sees the door or not

        Placeholder function of the webcam/depth camera
        """
        if self.verbose:
            print(f"\n\nCurrent position: {self.nodes[self.current_node][0], self.nodes[self.current_node][1]}")
            print(f"Goal position: {self.goal[0], self.goal[1]}")

        x = self.goal[0] - self.nodes[self.current_node][0] 
        y = self.goal[1] - self.nodes[self.current_node][1]
        
        # CHANGED: Full 360 degree CW angle computation
        angle_door_rad = math.atan2(y, x)
        angle_door = (math.degrees(angle_door_rad) + 360) % 360
        distance_door = math.sqrt(x**2 + y**2)

        if self.verbose:
            print(f"The door is at {distance_door}cm and {angle_door} degree")

        for point in ls_points:
            diff = (point.angle - angle_door) % 360
            
            # Only select the points that are in front of the door
            if diff < 0.1:
                if self.verbose:
                    print(f"The following point is on the way: {point} at coordinates: {(self.nodes[self.current_node][0] + (point.x / 10), self.nodes[self.current_node][1] + (point.y / 10))}")

                if (distance_door + 2 > point.distance/10):
                    if self.verbose:
                        print(f"Obstacle interference\n\n")

                    return False
        
        if self.verbose:
            print(f"No obstacle interference\n\n")

        return True

    def point_to_line_dist(self, p, a, b):
        """
        input: a point P and the two points A and B that forms a segment
        output: the distance between the point P and the segment formed by A, B (projection of P on the segment)

        Calculates the intersection point and the shortest distance 
        between point P and the line segment formed by A and B.
        """
        # 1. Extract coordinates (using index 3 and 4 for x, y from your lidar tuple)
        px, py = p[0], p[1]
        ax, ay = a[0], a[1]
        bx, by = b[0], b[1]


        # Find the function (slope and intercept) for line AB
        # Line: y = mx + c
        if bx - ax == 0:  # Vertical line case
            intersect_x = ax
            intersect_y = py
        elif by - ay == 0:  # Horizontal line case
            intersect_x = px
            intersect_y = ay
        else:
            # Slope of AB (m1)
            m1 = (by - ay) / (bx - ax)
            # Y-intercept of AB (c1 = y - mx)
            c1 = ay - m1 * ax

            # 2. Find the perpendicular function passing through P
            # Perpendicular slope (m2) is the negative reciprocal: -1/m1
            m2 = -1 / m1
            # Y-intercept of perpendicular line (c2 = y - mx)
            c2 = py - m2 * px

            # 3. Find the intersection point (cross point)
            # Set equations equal: m1*x + c1 = m2*x + c2
            # x(m1 - m2) = c2 - c1
            intersect_x = (c2 - c1) / (m1 - m2)
            intersect_y = m1 * intersect_x + c1


        max_x = max([ax, bx])
        min_x = min([ax, bx])
        max_y = max([ay, by])
        min_y = min([ay, by])

        if (min_x < intersect_x < max_x and min_y < intersect_y < max_y):
            distance = math.sqrt((px - intersect_x)**2 + (py - intersect_y)**2)

        else:
            distance = min([math.sqrt((px - ax)**2 + (py - ay)**2), math.sqrt((px - bx)**2 + (py - by)**2)])

        return distance
    
    def is_path_safe(self, start_pos, end_pos, ls_cluster, safety_margin=1.5):
        """
        input: a starting point, the point you want to go to, a list of cluster and a safety margin
        output: A boolean and a tupple of two coordinate. The first coordinate is the extremety, and the second is the neighbour of the extremety

        If the path is safe, no need to generate new node, just go there directly. 
        """
        safe_dist = self.robot_radius * safety_margin
        
        for cluster in ls_cluster:
            extrem = self.find_extremeties(cluster)
            for i in range(len(extrem)):
                px = self.nodes[self.current_node][0] + (extrem[i].x / 10)
                py = self.nodes[self.current_node][1] + (extrem[i].y / 10)
                
                dist = self.point_to_line_dist((px, py), start_pos, end_pos)
                
                if dist < safe_dist:
                    if self.verbose:
                        print(f"Minimum distance collision: {safe_dist}\tObserved distance: {dist}")

                    result = None

                    for p in cluster:
                        diff_angle = abs(p.angle - extrem[i].angle)

                        if diff_angle < 1 or diff_angle > 3:
                            # print(f"The angle is too small or too big for the point: {p} with dif angle {diff_angle}")
                            continue

                        min_dist = min(p.distance, extrem[i].distance)
                        expected_gap = min_dist * math.radians(diff_angle)
                        adaptive_delta = (expected_gap * 5) + 5
                        
                        if self.euclidean_distance(p, extrem[i]) < adaptive_delta:
                            result = p

                    return False, (extrem[i], result)
                
        return True, (None, None)
    
    def is_cluster_safe(self, p, ls_cluster, safety_margin=1.5):
        """
        input: a LidarPoint p (the coordinate of p must be local with respect to the lidar), a list of cluster and a safety margin
        output: boolean

        returns if the distance between your point p and all points in the list of cluster not part of the same cluster as p, is safe
        """

        safe_dist = self.robot_radius * safety_margin * 2

        for cluster in ls_cluster:
            if p in cluster:
                continue

            for c in cluster:
                new_dist = self.euclidean_distance(p, c)

                if new_dist < safe_dist:
                    return False
                
        return True
   
    def go_to(self, idx_node, ls_cluster, safety_margin=1.5):
        """
        input: index of the node, a list of cluster and a safety margin
        output: the ordered list of tuples (x, y, distance, angle) it will go through in the path

        It returns the path that the robot should follow going from current node to next node. 
        """
        start_pos = (self.nodes[self.current_node][0], self.nodes[self.current_node][1])
        end_pos = (self.nodes[idx_node][0], self.nodes[idx_node][1])

        self.remove_edge_between(self.current_node, idx_node)

        stack = [(start_pos, end_pos)]
        ls_pts_path = []
        ls_nodes = [start_pos]
        iteration = 0

        while stack:
            if iteration > 5:
                return None

            current_path = stack.pop(-1)
            iteration += 1

            if self.verbose:
                print(f"\n\nTrying to go from {current_path[0]} to {current_path[1]}")

            result, (p, point) = self.is_path_safe(current_path[0], current_path[1], ls_cluster, safety_margin)

            if not result:
                if self.verbose:
                    print(f"The path is not safe, collision at point {p} with coordinates: {(self.nodes[self.current_node][0] + (p.x / 10), self.nodes[self.current_node][1] + (p.y / 10))}")
                    
                for i in range(len(ls_cluster)):
                    if p in ls_cluster[i]:
                        if self.verbose:
                            print(f"The point is part of the cluster {i}")
                        break

                if self.verbose:
                    display_clusters_scatter(ls_cluster, [((self.nodes[idx_node][0] - self.nodes[self.current_node][0]) * 10, (self.nodes[idx_node][1] - self.nodes[self.current_node][1]) * 10), (p.x, p.y)])

                if not self.is_cluster_safe(p, ls_cluster, safety_margin):
                    return None
                    
                x = self.nodes[self.current_node][0] + (p.x / 10)
                y = self.nodes[self.current_node][1] + (p.y / 10)

                if self.verbose:
                    print(f"Computing the direction of the vector to avoid the collision point")
                    print(f"Point of collision at coordinates {x, y}")
                    print(f"Neighbour points to the collision: {point} with coordinates {(self.nodes[self.current_node][0] + (point.x / 10), self.nodes[self.current_node][1] + (point.y / 10))}")
                
                angle = math.atan2(y - current_path[0][1], x - current_path[0][0])

                if p.angle < point.angle:
                    angle -= np.pi / 2
                else:
                    angle += np.pi / 2
                
                distance = self.robot_radius * (safety_margin + 0.2)
                if self.verbose:
                    print(f"The direction of the vector is: {angle / np.pi * 180}\tThe distance is {distance}")

                new_point = (x + (np.cos(angle)*distance), y + (np.sin(angle)*distance))

                if self.verbose:
                    print(f"New intermediate point: {new_point}")

                stack.append((new_point, current_path[1]))
                stack.append((current_path[0], new_point))

            else:
                if self.verbose:
                    print(f"The path is safe")
                ls_nodes.append(current_path[1])


        # print(f"The path we are gonna follow: {ls_nodes}")
        ls_nodes[0] = self.current_node
        ls_nodes[-1] = idx_node

        for i in range(1, len(ls_nodes) - 1):
            self.add_node(ls_nodes[i][0], ls_nodes[i][1], True, True)
            ls_nodes[i] = len(self.nodes) - 1
            self.add_edge(ls_nodes[i-1], ls_nodes[i])

        self.add_edge(ls_nodes[-2], ls_nodes[-1])

        for i in range(len(ls_nodes) - 1):
            distance = self.distance(ls_nodes[i], ls_nodes[i+1])
            angle = self.angle(ls_nodes[i], ls_nodes[i+1])
            ls_pts_path.append((self.nodes[ls_nodes[i+1]][0], self.nodes[ls_nodes[i+1]][1], distance, angle))

        self.current_node = idx_node
        self.nodes[self.current_node] = (
            self.nodes[self.current_node][0],
            self.nodes[self.current_node][1],
            True,
            False
        )

        return ls_pts_path

    def list_child(self, index):
        """
        input: the index node
        output: the child of that node

        Returns the list of children of that node
        """
        
        ls_child = []

        for e in self.edge:
            if (e[0] == index):
                if (not self.nodes[e[1]][2]):
                    ls_child.append(e[1])

        return ls_child
    
    def go_back(self, safety_margin=1.5):
        """
        input: a list of cluster point and a safety margin
        output: returns the list of tuples (x, y, distance, angle) in the path it has to go

        If the robot meets a deadend, instead of generating new nodes (create an infinite tree), it goes back to its track using BFS,
        searching for the next unvisited node (closest in terms of tree). Once it reaches that node, it will scan and then apply go_to. 
        """

        ls_path = []
        done = False

        while (not done):

            if (self.current_node == 0):
                raise Exception("All nodes have been explored")
            
            # If have intermediate node
            ls_parents = [self.current_node, self.find_parent(self.current_node)]
            while self.nodes[ls_parents[-1]][3]:
                ls_parents.append(self.find_parent(ls_parents[-1]))
            
            for i in range(len(ls_parents) - 1):
                distance = self.distance(ls_parents[i], ls_parents[i+1])
                angle = self.angle(ls_parents[i], ls_parents[i+1])
                ls_path.append((self.nodes[ls_parents[i+1]][0], self.nodes[ls_parents[i+1]][1], distance, angle))

            self.current_node = ls_parents[-1]
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
            ls_path.append((self.getCoords(ls_nodes[i])[0], self.getCoords(ls_nodes[i])[1], distance, angle))

            self.current_node = ls_nodes[i]

        new_ls_points = self.sensor.scan(self.getCoords(self.current_node))
        new_ls_cluster = self.cluster(new_ls_points)

        goto_result = self.go_to(idx_node, new_ls_cluster, safety_margin) 
        
        # Prevent go_back crashes if the path becomes unexpectedly blocked
        if goto_result is not None:
            ls_path += goto_result
        else:
            self.nodes[idx_node] = (self.nodes[idx_node][0], self.nodes[idx_node][1], True, self.nodes[idx_node][3])
            if self.verbose:
                print(f"Could not reach {idx_node}, marking as visited.")

        return ls_path

    def run(self, safety_margin: float = 1.5):
        """
        input: a safety margin 
        output: the final list of tuples (x, y, distance, angle) that the robot will go to, basically running and getting to the final goal

        Main function. Basically puts everything together so the path finding algorithm works. 
        """
        x, y, _, _ = self.nodes[self.current_node]
        
        # Track where we are
        if (x, y) not in self.traversed_path:
            self.traversed_path.append((x, y))

        ls_point = self.sensor.scan((x, y))
        
        # Save the environment to memory!
        self.update_global_map(ls_point) 

        # Initialize the dictionary if it doesn't exist
        if not hasattr(self, 'visit_counts'):
            self.visit_counts = {}

        # Increment visit count for the current node
        self.visit_counts[self.current_node] = self.visit_counts.get(self.current_node, 0) + 1
        
        # If we have tried to find a path from here more than 3 times, 
        # it means we are stuck or in a micro-loop. Backtrack!
        if self.visit_counts[self.current_node] > 3:
            if self.verbose:
                print(f"Stuck at Node {self.current_node}. Backtracking...")
            return self.go_back(safety_margin)


        if self.verbose:
            print(f"Number of points detected: {len(ls_point)}")
            display_points(ls_point)

        ls_cluster = self.cluster(ls_point)
        if self.verbose:
            print(f"Number of obstacle: {len(ls_cluster)}")


        if (self.find_door(ls_point)):
            if self.verbose:
                print(f"Door seen!!")
            self.add_node(self.goal[0], self.goal[1])
            return self.go_to(len(self.nodes)-1, ls_cluster, safety_margin)

        
        if (len(ls_cluster) == 0):
            ls_choice = [-600, 0, 600]
            delta_x = np.random.choice(ls_choice)
            delta_y = np.random.choice(ls_choice)

            self.add_node(self.getCoords(self.current_node)[0] + delta_x, self.getCoords(self.current_node)[1] + delta_y, True, False)
            self.add_edge(self.current_node, len(self.nodes)-1)
            distance = self.distance(self.current_node, len(self.nodes)-1)
            angle = self.angle(self.current_node, len(self.nodes)-1)

            self.current_node = len(self.nodes)-1

            return [(self.getCoords(self.current_node)[0], self.getCoords(self.current_node)[1],distance, angle)]

        if self.verbose:
            print("Finding new nodes...")
        ls_nodes = self.find_new_node(ls_cluster)
        
        if self.verbose:
            ls_p = [((self.nodes[i][0] - self.nodes[self.current_node][0]) * 10, (self.nodes[i][1] - self.nodes[self.current_node][1]) * 10) for i in ls_nodes]
            display_clusters_scatter(ls_cluster, ls_p)


        result = None
        while result is None:
            if len(ls_nodes) == 0:
                return self.go_back(safety_margin)

            else:
                idx = np.random.choice(range(len(ls_nodes)))
                #idx = int(input(f"Enter the index between 0 and {len(ls_nodes) - 1}: "))
                result = self.go_to(ls_nodes[idx], ls_cluster, safety_margin)

                ls_nodes.pop(idx)
                    
                if len(ls_nodes) == 0 and result is None:
                    if self.verbose:
                        print("Deadend detected")
                    return self.go_back(safety_margin)

        return result

    def update_global_map(self, ls_point):
        """
        input: a list of LidarPoints
        output: None

        Updates the global map (add the global coordinates of the Lidarpoint to the list)
        """
        curr_x, curr_y, _, _ = self.nodes[self.current_node]
        
        for p in ls_point:
            global_x = curr_x + (p.x / 10)
            global_y = curr_y + (p.y / 10) 
            
            self.global_map_points.append((global_x, global_y))
