from robotDDR_enhance import Robot, robotGaphics
from RRT_base_enhance import RRT, RRTGraphics
from env_lidar import Fakelidar
import math
import pygame
import time

# Visualization parameters
MapW, MapH = 733, 475
MapDimensions=(MapH, MapW)
start = (680, 260)
goal = (20, 404)
clock = pygame.time.Clock()     # timing for robot movement
dt = 0                          # Distance that the robot will move for one action
iteration = 0                   # iteration for RRT
step_before_seeing_goal = 100
robot_radius = 5


# Setup window and screen
pygame.display.set_caption('DDR Robot')
screen = pygame.display.set_mode((MapW, MapH))
screen.fill((0,0,0))

# Map 
mapImg_path = "star.webp" # 'map1.png'
roomMap = pygame.image.load(mapImg_path)
screen.blit(roomMap, (0,0))
sensor = Fakelidar(mapImg_path, goal)


# Robot
robot = Robot(start)
RobotGraphics = robotGaphics(robot, screen, MapDimensions, robot_img_path='robot.png')

# RRT
rrt_path = RRT(start, goal, MapDimensions, robot_radius, sensor)
rrt_Graphics = RRTGraphics(rrt_path, screen, MapDimensions)


running = True
while running: 
    # update time
    clock.tick_busy_loop(60)
    dt = (clock.get_time() - dt) / 1000

    # Stop upon x press
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Reset screen
    screen.fill((255,255,255))
    screen.blit(roomMap, (0,0))

    # Include the bias
    if iteration == step_before_seeing_goal:
        rrt_path.spotted_goal = True
        print("Goal spotted")

    # Run RRT while path not found
    if not rrt_path.path_to_goal():
        rrt_path.run_RRT(iteration)

        rrt_Graphics.updateRRTGraphics()

        iteration += 1

    # Run robot once path is found
    else:
        # load path into robot memory
        if not robot.path:
            print(f"rrt path: {rrt_path.getPathCoords()}")
            robot.path = rrt_path.getPathCoords().copy()
            robot.path.reverse()
            print(f"robot path: {robot.path}")

        robot.moveRobot(dt)
        RobotGraphics.updateRobot()
    RobotGraphics.drawRobot()

    pygame.display.update()