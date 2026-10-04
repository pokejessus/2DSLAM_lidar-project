import time

import numpy as np
import pygame
import matplotlib.pyplot as plt
import math

from path_finding_alg import path_finding
from path_finding_graphics import Robot
from env_lidar import Fakelidar, display_points

# Simulation variable
startPosition = (1400, 750)
goalPosition = (130, 740)
mapImg_path = "demo.png" # 'map1.png'
mapDimension = (1728, 1080)
robotRadius = 15
safety_margin = 1.1

pygame.init()
clock = pygame.time.Clock()
iteration = 0

# Display variables
pygame.display.set_caption("Exploring Robot")
screen = pygame.display.set_mode(mapDimension) # Set to the size of the image
screen.fill((0, 0, 0))


# Map 
roomMap = pygame.image.load(mapImg_path)
screen.blit(roomMap, (0,0))
sensor = Fakelidar(mapImg_path, goalPosition)


robot = Robot(*startPosition)
alg = path_finding(startPosition, goalPosition, mapDimension, robotRadius, sensor, verbose=False)

print(f"Min robot distance: {robotRadius * 1.5}")

robot.update()

running = True
while running:
    # Stop upon x press
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                iteration += 1
                temp = alg.run(safety_margin)
                #print(f"Path that we are gonna follow: {temp}")
                robot.set_path(temp)

    robot.update()
    # 2. NEW: Save the robot's actual position to the path memory every frame
    # We only add it if the robot has actually moved to save memory
    current_pos = (robot.x, robot.y)
    if not alg.traversed_path or alg.traversed_path[-1] != current_pos:
        alg.traversed_path.append(current_pos)

    # 3. Goal check
    if math.hypot(robot.x - goalPosition[0], robot.y - goalPosition[1]) < 5:
        print("Goal Reached!")
        running = False

    # 4. DRAWING (Background -> Memory -> Robot)
    screen.fill((255,255,255))
    screen.blit(roomMap, (0,0))

    # Draw the mental map (dots)
    for point in alg.global_map_points:
        pygame.draw.circle(screen, (120, 120, 120), (int(point[0]), int(point[1])), 1)

    # Draw the smooth path (the breadcrumbs)
    if len(alg.traversed_path) > 1:
        # We use a thickness of 2 or 3 to make it visible
        pygame.draw.lines(screen, (0, 200, 0), False, alg.traversed_path, 2)

    for node in alg.nodes:
        # node[0] is x, node[1] is y
        pygame.draw.circle(screen, (0, 0, 255), (int(node[0]), int(node[1])), 5)
    
    # D. NEW: Draw the Current Active Node (Yellow circle)
    # This helps you see which node the robot is currently "thinking" from
    curr_node_coords = alg.nodes[alg.current_node]
    pygame.draw.circle(screen, (255, 255, 0), (int(curr_node_coords[0]), int(curr_node_coords[1])), 8)

    robot.draw(screen)
    pygame.display.update()
    #     pygame.display.flip()
    clock.tick(60) # 60 FPS

# Saves the final map
print("Saving final map...")
pygame.image.save(screen, "final_exploration_map.png")

# pygame.quit()

