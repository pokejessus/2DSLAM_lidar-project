import numpy as np
import math
import matplotlib.pyplot as plt
from test_driver_lidar import LidarPoint
from PIL import Image


class Fakelidar:
    def __init__(self, path: str, pos_goal: tuple, perf_accuracy: bool = False):
        img = Image.open(path).convert('L') #convert('1')
        self.img_array = np.array(img, dtype = int)
        self.img_array -= 255
        self.img_array *= -1
        self.pos_goal = pos_goal
        self.perf_accuracy = perf_accuracy

        print(self.img_array)

    def laser(self, angle: int, robotPosition: tuple):
        """
        Angle is in degrees.
        robotPosition is a tuple with coordinates in pixels (x, y).
        """
        new_angle = angle * (np.pi / 180)

        # Raw direction vector (y is flipped in image coordinates)
        dx =  np.cos(new_angle)
        dy = -np.sin(new_angle)

        # Normalize so the dominant axis steps exactly 1 pixel per iteration.
        # This decomposes the big triangle into many small ones where
        # either |Δx| = 1 (horizontal-dominant) or |Δy| = 1 (vertical-dominant).
        if abs(dx) >= abs(dy):          # horizontal-dominant
            step_x = np.sign(dx)        # always ±1
            step_y = dy / abs(dx)       # fractional  (|step_y| ≤ 1)
        else:                           # vertical-dominant
            step_y = np.sign(dy)        # always ±1
            step_x = dx / abs(dy)       # fractional  (|step_x| ≤ 1)

        current_x = float(robotPosition[0])
        current_y = float(robotPosition[1])

        while True:
            current_x += step_x
            current_y += step_y

            ix = int(round(current_x))
            iy = int(round(current_y))

            # Out-of-bounds check
            if not (0 <= ix < self.img_array.shape[1] and
                    0 <= iy < self.img_array.shape[0]):
                return None

            # Obstacle check
            if self.img_array[iy, ix] != 0:
                return ix, iy

    def scan(self, robotPosition: tuple):
        
        result = []
        for i in np.arange(0.0, 360.0, 0.1):
            r = self.laser(i, robotPosition)

            if (r is None):
                continue
            x, y = r

            distance = 10 * np.sqrt((x - robotPosition[0])**2 + (y - robotPosition[1])**2)

            if (distance > 12000):
                continue

            if (not self.perf_accuracy):
                result.append(LidarPoint(i, distance + np.random.random()*distance/200-distance/400, self.img_array[y, x]))

            result.append(LidarPoint(i, distance + np.random.random(), self.img_array[y, x]))

        return result
    

    def locateGoal(self, robotPosition: tuple):
        distance = np.sqrt((self.pos_goal[0]- robotPosition[0])**2 + (self.pos_goal[1] - robotPosition[1])**2)
        y = self.pos_goal[1] - robotPosition[1]
        x = self.pos_goal[0] - robotPosition[0]

        angle = np.arctan(y/(x if x != 0 else 1e-6))*(180/np.pi)

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

"""
def t1(pos_robot, pos_goal):
    x_distance = pos_goal[0] - pos_robot[0]
    y_distance = pos_goal[1] - pos_robot[1]
    angle_goal = np.arctan(y_distance/x_distance)

    print(angle_goal)
"""

def check_collision_test(position: tuple, ls_lidar_point: list, goal: tuple, radius_robot: float) -> bool:
    if not ls_lidar_point:
        return False

    A = np.array(position)
    B = np.array(goal)
    path_vec = B - A
    path_len_sq = np.dot(path_vec, path_vec)

    # 1. Reconstruct absolute obstacle coordinates directly from distance/angle
    # Convert mm directly to cm
    distances_cm = np.array([p.distance for p in ls_lidar_point]) / 10.0
    angles_rad = np.radians(np.array([p.angle for p in ls_lidar_point]))
    
    # 2. Apply the inverted Y-axis logic. 
    # Standard cos() for X, but we must use -sin() for Y so that 90 degrees 
    # results in a negative offset (moving "up" in the array).
    rel_x = distances_cm * np.cos(angles_rad)
    rel_y = -distances_cm * np.sin(angles_rad) 
    
    # 3. Create the (N, 2) matrix of absolute obstacle positions
    # We add the robot's current position (A) to the relative offsets
    P = np.column_stack((A[0] + rel_x, A[1] + rel_y))

    # 4. Handle edge case where robot is already at the goal
    if path_len_sq < 1e-8:
        dists_to_start = np.linalg.norm(P - A, axis=1)
        return np.any(dists_to_start <= radius_robot)

    # 5. Calculate projection and check for collisions
    robot_to_points = P - A
    t = np.dot(robot_to_points, path_vec) / path_len_sq
    t_clamped = np.clip(t, 0.0, 1.0)
    
    closest_points = A + t_clamped[:, np.newaxis] * path_vec
    dists = np.linalg.norm(P - closest_points, axis=1)
    
    return np.any(dists <= radius_robot)


def display_points (ls_points):
    max_x = 0
    min_x = 0
    max_y = 0
    min_y = 0
    for point in ls_points:
        if (point.x > max_x):
            max_x = point.x
        if (point.x < min_x):
            min_x = point.x
        if (point.y > max_y):
            max_y = point.y
        if (point.y < min_y):
            min_y = point.y

    # Creating the dimension of the rectangle
    dim =  (int(np.max([np.abs(min_y), np.min(max_y)]) // 5 + 1), int(np.max([np.abs(min_x), np.abs(max_x)]) // 5 + 1))
    img = np.zeros(dim)

    pos_robot = (img.shape[0]//2, img.shape[1]//2)

    for point in ls_points:
        img[pos_robot[0] + int(point.y//10), pos_robot[1] + int(point.x//10)] = point.intensity

    plt.imshow(img)

    plt.show()




if __name__ == "__main__":

    #"""
    #robot_point = (0,0)
    #goal_point = (45, 45)
    #LidarPoint_ls = [LidarPoint(90,282,20)]
    #radius = 10

    #print(check_collision_test(robot_point, LidarPoint_ls, goal_point, radius))
    #"""
    r_pos = (85, 50)
    g_pos = (130, 50)

    A = Fakelidar("right.png",g_pos, False) # "C:\\Users\jessi\\OneDrive\\Pictures\\McGill\\star.webp"

    print(A.locateGoal(r_pos))

    ls_points = A.scan(r_pos)

    ls = sorted(ls_points, key=lambda p: p.distance)

    print(check_collision_test(r_pos, ls, g_pos, 10))

    display_points(ls)

    #for i in ls[:20]:
    #    print(f"{i}, {i.x}, {i.y}")
