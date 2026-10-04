import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.cm as cm

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

    def laser(self, angle: int | float | np.floating, robotPosition: tuple):
        """
        Angle is in degrees.
        robotPosition is a tuple with coordinates in pixels (x, y).
        """
        new_angle = angle * (np.pi / 180)

        # Raw direction vector (y is flipped in image coordinates)
        dx =  np.cos(new_angle)
        dy =  np.sin(new_angle)

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

def display_points(ls_points, scale=10):
    if not ls_points:
        print("No points to display.")
        return

    # 1. Easily find min and max values using Python's built-in functions
    xs =[point.x for point in ls_points]
    ys = [point.y for point in ls_points]
    
    # We find the maximum absolute distance from the origin on both axes
    # to ensure the robot is perfectly centered in the final image.
    max_abs_x = max(abs(max(xs)), abs(min(xs)))
    max_abs_y = max(abs(max(ys)), abs(min(ys)))

    # 2. Create the dimensions of the grid
    # We multiply by 2 for positive/negative space, divide by scale, and add padding
    padding = 10
    height = int((max_abs_y / scale) * 2) + padding
    width = int((max_abs_x / scale) * 2) + padding

    img = np.zeros((height, width))

    # 3. Place the robot exactly in the center of the newly created image
    pos_robot = (height // 2, width // 2)

    # 4. Map the points
    for point in ls_points:
        # Map X and Y appropriately:
        # - Subtracting Y: moves positive Y values UP the image matrix.
        # + Adding X: moves positive X values RIGHT across the image matrix.
        row = pos_robot[0] + int(point.y / scale)
        col = pos_robot[1] + int(point.x / scale)

        # 5. Draw the point safely (ignoring any out-of-bounds floating errors)
        if 0 <= row < height and 0 <= col < width:
            img[row, col] = point.intensity

    # 6. Display the output
    # cmap='gray' is recommended for lidar intensity visualization
    plt.imshow(img, cmap='gray')
    plt.title("Lidar Point Cloud")
    plt.show()

def display_points_scatter(ls_points):
    if not ls_points:
        print("No points to display")
        return
    
    plt.figure(figsize=(10, 8))
    
    ls_x = [point.x for point in ls_points]
    ls_y = [point.y for point in ls_points]

    plt.scatter(ls_x, ls_y)
    plt.title("Obstacle detected by the Lidar")
    plt.axis("equal")
    plt.show()




def display_clusters_image(ls_cluster, scale=10):
    if not ls_cluster:
        print("No clusters to display.")
        return

    # 1. Find the bounds using all points in all clusters
    all_points = [p for cluster in ls_cluster for p in cluster]
    xs = [p.x for p in all_points]
    ys =[p.y for p in all_points]
    
    max_abs_x = max(abs(max(xs)), abs(min(xs)))
    max_abs_y = max(abs(max(ys)), abs(min(ys)))

    # 2. Setup image dimensions
    padding = 10
    height = int((max_abs_y / scale) * 2) + padding
    width = int((max_abs_x / scale) * 2) + padding

    # Create an RGB image (Height, Width, 3 color channels) initialized to black
    img = np.zeros((height, width, 3), dtype=np.float32)
    pos_robot = (height // 2, width // 2)

    # Generate colors (hsv colormap generates vivid distinct colors)
    # This returns an RGBA color, so we slice[:3] to get just RGB
    colors = plt.cm.hsv(np.linspace(0, 1, len(ls_cluster))) 

    # 3. Map the points
    for i, cluster in enumerate(ls_cluster):
        cluster_color = colors[i][:3] # Extract RGB
        
        for point in cluster:
            row = pos_robot[0] - int(point.y / scale)
            col = pos_robot[1] + int(point.x / scale)

            if 0 <= row < height and 0 <= col < width:
                # Assign the specific color to the pixel
                img[row, col] = cluster_color

    # Draw the robot in the center as white
    img[pos_robot[0], pos_robot[1]] = [1.0, 1.0, 1.0]

    # 4. Show the image
    plt.imshow(img)
    plt.title(f"Detected Obstacles: {len(ls_cluster)}")
    plt.show()

def display_clusters_scatter(ls_cluster, ls_points = None):
    if not ls_cluster:
        print("No clusters to display.")
        return

    # Made the figure slightly wider to accommodate the legend on the right
    plt.figure(figsize=(10, 8))
    
    # Generate a unique color for each cluster
    colors = cm.rainbow(np.linspace(0, 1, len(ls_cluster)))
    
    for i, cluster in enumerate(ls_cluster):
        xs =[point.x for point in cluster]
        ys = [point.y for point in cluster]
        
        # The 'label' parameter is what the legend will read
        plt.scatter(xs, ys, color=colors[i], s=10, label=f'Cluster {i}')
    
    if ls_points is not None:
        for i, (x, y) in enumerate(ls_points):
            plt.scatter(x, y, label=f"Point between {i, (i + 1) % len(ls_cluster)}")

    # Plot the robot's position at the origin (0, 0)
    plt.scatter(0, 0, color='black', marker='+', s=200, label='Robot (Lidar)')

    plt.title(f"Detected Obstacles: {len(ls_cluster)}")
    plt.xlabel("X (Right)")
    plt.ylabel("Y (for image)")
    
    # Ensures the 2D plane isn't stretched
    plt.axis('equal') 
    plt.gca().invert_yaxis()
    plt.grid(True, linestyle='--', alpha=0.5)
    
    # --- NEW: ADDING THE LEGEND ---
    # bbox_to_anchor pushes the legend outside the bounding box of the graph
    # markerscale=2 makes the dots in the legend slightly larger and easier to see
    plt.legend(title="Cluster Index", bbox_to_anchor=(1.05, 1), loc='upper left', markerscale=2)
    
    # tight_layout ensures the external legend doesn't get cut off the edge of the window
    plt.tight_layout() 
    
    plt.show()


if __name__ == "__main__":

    #"""
    #robot_point = (0,0)
    #goal_point = (45, 45)
    #LidarPoint_ls = [LidarPoint(90,282,20)]
    #radius = 10

    #print(check_collision_test(robot_point, LidarPoint_ls, goal_point, radius))
    #"""
    r_pos = (100, 65)
    g_pos = (130, 50)

    A = Fakelidar("corner.png",g_pos, False) # "C:\\Users\jessi\\OneDrive\\Pictures\\McGill\\star.webp"

    print(A.locateGoal(r_pos))

    ls_points = A.scan(r_pos)

    ls = sorted(ls_points, key=lambda p: p.distance)

    # print(check_collision_test(r_pos, ls, g_pos, 10))

    display_points_scatter(ls)

    #for i in ls[:20]:
    #    print(f"{i}, {i.x}, {i.y}")
