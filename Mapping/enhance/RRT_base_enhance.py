import random
import math
import pygame

from env_lidar import Fakelidar, check_collision_test

class RRT:
    def __init__(self, start, goal, MapDimensions, robot_radius, sensor: Fakelidar):
        # variables to run the algorithm
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.dmax = 35
        self.current_pos = 0
        self.spotted_goal = False
        self.robot_radius = robot_radius

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

    def add_node(self, x, y):
        '''
        x, y: coords
        '''
        self.nodes.append((x, y))

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
        self.edge.pop(edge)

    def number_of_nodes(self):
        return len(self.nodes)

    # Measure distance between two nodes that are adjacent
    def distance(self, n1, n2):
        x1, y1 = self.nodes[n1]
        x2, y2 = self.nodes[n2]
        px = (float(x2) - float(x1))**2 
        py = (float(y2) - float(y1))**2
        return math.sqrt(px + py)
    
    def sample_env(self):
        x = int(random.uniform(0, self.mapW))
        y = int(random.uniform(0, self.mapH))
        return x, y

    # Returns nearest node to new node
    def nearest(self, n):
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            dist = self.distance(i, n)
            if  dist < dmin:
                dmin = dist 
                nnear = i
        return nnear 
    
    """
    def onWall(self, x, y):
        # check pixel color
        pixel_color = self.roomMap.get_at((x, y))
        # print(f"onWall(): color of pixel at ({x}, {y}) is {pixel_color}")
        if pixel_color[0] <= 5 and pixel_color[1] <= 5 and pixel_color[2] <= 5:
            return True
        return False

    def isFree(self):
        # ix and pos of newly added node
        n = self.number_of_nodes() - 1
        x, y = self.nodes[n]

        if self.onWall(x, y):
            # print(f"isFree() checking if node n is on wall")
            self.remove_node(n)
            return False
        return True

    # Check if an edge between nodes is between obstacles
    def crossObstacle(self, x1, x2, y1, y2):
        # look 100 steps between the two nodes
        for i in range(0, 101):
            u = i/100
            x = x1*u + x2*(1-u)
            y = y1*u + y2*(1-u)

            if self.onWall(int(x), int(y)):
                # print("Wall was detected between nodes when trying to connect")
                return True
        return False
    """
        
    # Add an edge between nodes n1 and n2
    def connect(self, n1, n2):
        ls_points = self.sensor.scan(self.nodes[n1])
        #print(ls_points)

        if check_collision_test(self.nodes[n1], ls_points, self.nodes[n2], self.robot_radius):      #self.crossObstacle(x1, x2, y1, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            return True
        
    

    # Check if we reached the goal
    def step(self, nnear, nrand):
        d = self.distance(nnear, nrand)
        if d > self.dmax:
            # add a node at max mistance self.dmax in the direction of node to add
            xnear, ynear = self.getCoords(nnear)
            xrand, yrand = self.getCoords(nrand)
            px = xrand - xnear
            py = yrand - ynear
            theta = math.atan2(py, px)
            x, y = int(xnear + self.dmax * math.cos(theta)), int(ynear + self.dmax * math.sin(theta))
            self.remove_node(nrand)
            
            # if reached end, add last node on coordinates of end
            if abs(x - self.goal[0]) < self.dmax and abs(y - self.goal[1]) < self.dmax:
                self.add_node(self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.add_node(x, y)

    def find_parent(self, n1_index):
        if (n1_index == 0):
            return 0
        for edge in self.edge:
            if edge[1] == n1_index:
                return edge[0]
            

    def path_to_root(self, n1_index):
        if (n1_index == 0):
            return []

        path_root = []
        path_root.append(n1_index)
        newpos = self.find_parent(n1_index)
        
        while (newpos != 0):
            path_root.append(newpos)
            newpos = self.find_parent(newpos)
        
        path_root.append(0)
        return path_root

    # Traverse back through parents once the goal is reached
    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.find_parent(self.goalstate)
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.find_parent(newpos)
            self.path.append(0)

        return self.goalFlag

    # combine both paths
    def path_robot_to_nnear(self, path_robot, path_nnear):
        if len(path_robot) == 0:
            return path_nnear
        if len(path_nnear) == 0:
            return path_robot

        path_nnear_reverted = []
        result = []
        for index in path_nnear:
            path_nnear_reverted.insert(0,index)

        for index in path_robot:
            result.append(index)
            if index in path_nnear_reverted:
                break

        while (result[-1] != path_nnear_reverted[0]):
            path_nnear_reverted.pop(0)
            
        path_nnear_reverted.pop(0)

        for index in path_nnear_reverted:
            result.append(index)

        return result

    # Get coordinates of path once path_to_goal() has run
    def getPathCoords(self):
        """
        return the list of coordinates from the goal to root
        
        :param self: 
        """
        return self._get_path_coordinates(self.path)

    def _get_path_coordinates(self, path):
        pathCoords = []
        for node in path:
            pathCoords.append(self.getCoords(node))
        return pathCoords

    # returns path between robot and nearest node to the goal
    def bias_1(self):
        self.add_node(self.goal[0], self.goal[1])
        n = self.number_of_nodes() - 1
        xnearest = self.nearest(n)

        root_path = self.path_to_root(xnearest)
        robot_path = self.path_to_root(self.current_pos)
        final_path = self.path_robot_to_nnear(robot_path, root_path)

        return self._get_path_coordinates(final_path)
    
    # create a new node to the direction of the goal
    def bias_2(self):
        n = self.number_of_nodes() - 1
        self.step(self.current_pos, n)
        # print(f"expand(): want to connect {xnearest} ({self.x[xnearest], self.y[xnearest]}) with {n} ({self.x[n], self.y[n]})")
        #if self.isFree():
            # print(f"expand(): connecting {xnearest} ({self.x[xnearest], self.y[xnearest]}) with {n} ({self.x[n], self.y[n]})")
        if (not self.connect(self.current_pos, n)) and (self.goalFlag):
            self.goalFlag = False
        
        return []
        
    # random expansion of tree using random samples
    """
    def expand(self):   
        n = self.number_of_nodes()
        x, y = self.sample_env()
        self.add_node(x, y)
        xnearest = self.nearest(n)
        self.step(xnearest, n)
        # print(f"expand(): want to connect {xnearest} ({self.x[xnearest], self.y[xnearest]}) with {n} ({self.x[n], self.y[n]})")
        if self.isFree():
            # print(f"expand(): connecting {xnearest} ({self.x[xnearest], self.y[xnearest]}) with {n} ({self.x[n], self.y[n]})")
            self.connect(xnearest, n)
    """
    def expand1(self):
        n = self.number_of_nodes()
        x, y = self.sample_env()
        self.add_node(x, y)
        xnearest = self.nearest(n)

        l1 = self.path_to_root(self.current_pos)
        l2 = self.path_to_root(xnearest)

        self.current_pos = xnearest
        result = self.path_robot_to_nnear(l1, l2)

        return self._get_path_coordinates(result)
        

    def expand2(self):
        n = self.number_of_nodes() - 1
        self.step(self.current_pos, n)
        # print(f"expand(): want to connect {xnearest} ({self.x[xnearest], self.y[xnearest]}) with {n} ({self.x[n], self.y[n]})")
        #if self.isFree():
            # print(f"expand(): connecting {xnearest} ({self.x[xnearest], self.y[xnearest]}) with {n} ({self.x[n], self.y[n]})")
        if (not self.connect(self.current_pos, n)) and (self.goalFlag):
            self.goalFlag = False
        
        return []

    def find_goal(self, goal):
        self.spotted_goal = True
        self.goal = goal

    def run_RRT(self, iter):
        # every iteration, either bias or expand (want 10-90 ratio)
        
        if iter % 20 == 0 and self.spotted_goal:
            return self.bias_1()
        elif(iter % 20 == 1 and self.spotted_goal):
            return self.bias_2()
        elif (iter % 2 == 0):
            return self.expand1()
        else:
            return self.expand2()
        
    

    # # restarts alrgorithm with current position as new starting point
    # def restartRRT(self)



class RRTGraphics:
    def __init__(self, rrt_path: RRT, screen, MapDimensions):
         # Initialize pygame
        pygame.init()

        # Dimensions and Screen
        self.screen = screen
        self.MapDimensions = MapDimensions
        self.mapH, self.mapW = MapDimensions

        # RRT path object
        self.rrt_path = rrt_path

        # Node and Edge graphics
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1

    def showStart_Goal(self):
        """
        Draw the goal on the picture
        """
        pygame.draw.circle(self.screen, (0,255,0), self.rrt_path.start, 5, 0)
        pygame.draw.circle(self.screen, (255,0,0), self.rrt_path.goal, 10, 1)

    def drawNodes(self):
        """
        Draw all the nodes and the their connection
        """
        for i in range(self.rrt_path.number_of_nodes()):
            nodePos = self.rrt_path.getCoords(i)
            pygame.draw.circle(self.screen, (0,255,0), nodePos, self.nodeRad+2, 0)
            
            parentNodeIdx = self.rrt_path.find_parent(i)

            if parentNodeIdx is None:
                parentNodeIdx = i

            parentNodePos = self.rrt_path.getCoords(parentNodeIdx)

            pygame.draw.line(self.screen, (0,255,255), nodePos, parentNodePos, self.edgeThickness)

    def drawPath(self):
        """
        Draw the path from the starting point to the goal once we found it 
        """
        pathCoords = self.rrt_path.getPathCoords()
        for node in pathCoords:
            pygame.draw.circle(self.screen, (255,0,0), node, self.nodeRad+3, 0)

    def updateRRTGraphics(self):
        """
        Update the current graph
        """
        self.showStart_Goal()
        self.drawNodes()


if __name__ == "__main__":
    print("yolo")
    mapImg_path = "map1.png"
    goal = (20, 150) # (150, 20)
    r_pos = (60, 200) # (200, 60)

    sensor = Fakelidar(mapImg_path, goal)
    rrt = RRT(r_pos, goal, (2000, 2000), 10, sensor)

    rrt.add_node(*goal)
    print(rrt.connect(0, 1))

