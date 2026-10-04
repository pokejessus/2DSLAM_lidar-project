import numpy as np
import math
from test_driver_lidar import LidarPoint
from PIL import Image


class Fakelidar:
    def __init__(self, path: str, pos_goal: tuple):
        img = Image.open(path).convert('L') #convert('1')
        self.img_array = np.array(img, dtype = int)
        self.img_array -= 255
        self.img_array *= -1
        self.pos_goal = pos_goal

        print(self.img_array)

    def laser(self, angle: int, robotPosition: tuple):
        """
        Angle is in degree
        robot position is a tuple with the coordinates in mm
        """
        new_angle = angle*(np.pi/180)
        y = int(np.sin(new_angle)*100)
        x = int(np.cos(new_angle)*100)
        
        gcd = math.gcd(x, y)
        y = - y // gcd
        x = x // gcd

        current_x = robotPosition[0]
        current_y = robotPosition[1]

        while True:
            for i in range(np.abs(x)):
                if not (0 < (current_x + (i+1)*np.sign(x)) < self.img_array.shape[1]):
                    return None
                if (self.img_array[current_y, current_x + (i+1)*np.sign(x)] != 0):
                    return current_x + (i+1)*np.sign(x), current_y
            current_x += x
            
            for i in range(np.abs(y)):
                if not (0 < (current_y + (i+1)*np.sign(y)) < self.img_array.shape[0]):
                    return None
                if (self.img_array[current_y+ (i+1)*np.sign(y), current_x] != 0):
                    return current_x, current_y+ (i+1)*np.sign(y)
            current_y += y
            

    def scan(self, robotPosition: tuple):
        
        result = []
        for i in range(360):
            r = self.laser(i, robotPosition)

            if (r is None):
                continue

            x, y = r

            result.append(LidarPoint(i, 10 * np.sqrt((x - robotPosition[0])**2 + (y - robotPosition[1])**2), self.img_array[y, x]))

        return result
    

    def locateGoal(self, robotPosition: tuple):
        distance = np.sqrt((self.pos_goal[0]- robotPosition[0])**2 + (self.pos_goal[1] - robotPosition[1])**2)
        y = self.pos_goal[1] - robotPosition[1]
        x = self.pos_goal[0] - robotPosition[0]

        angle = np.arctan(y/x)*(180/np.pi)

        r = self.laser(angle, robotPosition)

        if (r is None):
            return self.pos_goal
        
        x, y = r 

        distance_obs = np.sqrt((x - robotPosition[0])**2 + (y - robotPosition[1])**2)

        if (distance_obs > distance):
            return self.pos_goal
        else:
            return None




"""
def check_collision(position: tuple, ls_lidar_point: list, goal: tuple, distance: float, radius_robot: float):

    cm_lidar_point_ls = []

    for point in ls_lidar_point:
        cm_lidar_point_ls.append((point.x/10, point.y/10))

    if (position == goal):
        return True
    
    x_distance = goal[0] - position[0]
    y_distance = goal[1] - position[1]
    angle_goal = np.arctan(y_distance/x_distance)

    theta_1 = angle_goal + np.pi/2
    theta_2 = angle_goal - np.pi/2
    
    rectangle_pt1 = (np.cos(theta_1)*radius_robot, np.sin(theta_1)*radius_robot)
    rectangle_pt2 = (np.cos(theta_2)*radius_robot, np.sin(theta_2)*radius_robot)
    rectangle_pt3 = (np.cos(angle_goal)*distance + rectangle_pt2[0], np.sin(angle_goal)*distance + rectangle_pt2[1])
    rectangle_pt4 = (np.cos(angle_goal)*distance + rectangle_pt1[0], np.sin(angle_goal)*distance + rectangle_pt1[1])
  
    a1 = (rectangle_pt4[1] - rectangle_pt1[1])/(rectangle_pt4[0] - rectangle_pt1[0])
    b1 = rectangle_pt1[1] - rectangle_pt1[0]*a1


    if (np.arctan(point[0]) < rectangle_pt1[0] or np.arctan(point[0]) > rectangle_pt3[0]
        or np.arctan(point[1]) < rectangle_pt2[1] or np.arctan(point[1]) > rectangle_pt4):
"""


def check_collision_test(position: tuple, ls_lidar_point: list, goal: tuple, radius_robot: float):

    cm_lidar_point_ls = []

    for point in ls_lidar_point:
        cm_lidar_point_ls.append((point.x/10, point.y/10))

    if (position == goal):
        return True

    A = np.array(position)
    B = np.array(goal)
    path_vec = B - A
    path_len_sq = np.dot(path_vec, path_vec)
    
    # Vectorize the obstacle points
    P = np.array(cm_lidar_point_ls) # matrix (N, 2)
    robot_to_points = P - A   # matrix (N, 2)
    
    t = np.dot(robot_to_points, path_vec) / path_len_sq #percentage
    
    # Find projection on the robot path
    projections = A + t[:, np.newaxis] * path_vec #find the point on the path closest to the point
    perp_dists = np.linalg.norm(P - projections, axis=1) #find the distance between the points (perp)
    
    # Return True if any point satisfies the collision condition
    collision_mask = (t >= 0) & (t <= 1) & (perp_dists <= radius_robot) # length x width
    return np.any(collision_mask)


"""
def t1(pos_robot, pos_goal):
    x_distance = pos_goal[0] - pos_robot[0]
    y_distance = pos_goal[1] - pos_robot[1]
    angle_goal = np.arctan(y_distance/x_distance)

    print(angle_goal)
"""



if __name__ == "__main__":

    """
    robot_point = (0,0)
    goal_point = (45, 45)
    LidarPoint_ls = [LidarPoint(0,0,20)]
    radius = 10

    print(check_collision_test(robot_point, LidarPoint_ls, goal_point, radius))
    """
    g_pos = (200, 150)
    r_pos = (150, 400)

    A = Fakelidar("C:\\Users\jessi\\OneDrive\\Pictures\\McGill\\star.webp",g_pos)

    print(A.locateGoal(r_pos))

    #ls_points = A.scan((10, 10))

    #ls = sorted(ls_points, key=lambda p: p.distance)

    #for i in ls:
    #    print(i)
