import pygame
import math
import numpy as np


class robotGaphics:
    def __init__(self, robot, screen, MapDimensions, robot_img_path='robot.png'):
        # Initialize pygame
        pygame.init()

        # Dimensions and Screen
        self.screen = screen
        self.MapDimensions = MapDimensions
        self.mapH, self.mapW = MapDimensions

        # Robot 
        self.robot = robot
        self.robot_img = pygame.image.load(robot_img_path)
        # self.img = pygame.transform.scale(self.img, (self.width, self.width))
        self.rotated = self.robot_img                                   # Deal with robot rotation
        self.rect = self.rotated.get_rect(center=(self.robot.x, self.robot.y))

        # Text for robot
        self.font = pygame.font.Font('freesansbold.ttf', 50)
        self.text = self.font.render('default', True,(255,255,255), (0,0,0))
        self.textRect = self.text.get_rect()
        self.textRect.center = (self.mapW - 600, self.mapH-100)

        # Trail
        self.trail_set = []

    def updateRobotGraphics(self):
        # Update graphics 
        self.rotated = pygame.transform.rotozoom(self.robot_img, math.degrees(-self.robot.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.robot.x, self.robot.y))

    def drawRobot(self):
        self.updateRobotGraphics()
        self.screen.blit(self.rotated, self.rect)  

    def write_info(self):
        txt = f"u = {self.robot.u:.1f} w = {self.robot.w:.1f} theta = {self.robot.theta:.2f}"
        self.text = self.font.render(txt, True, (255,255,255), (0,0,0))
        self.screen.blit(self.text, self.textRect)

    def drawTrail(self, trail_color):
        for i in range(0, len(self.trail_set)-1):
            pygame.draw.line(self.screen, trail_color, (self.trail_set[i][0], self.trail_set[i][1]),
                             (self.trail_set[i+1][0], self.trail_set[i+1][1]), 2)
        if len(self.trail_set) > 30000:
            self.trail_set.pop(0)
        self.trail_set.append((self.robot.x, self.robot.y))

    def drawPath(self):
        for p in self.robot.path:
            pygame.draw.circle(self.screen, (100,0,199), p, 3, 1)

    def updateRobot(self):
        # self.write_info()
        self.drawPath()
        # self.drawRobot()
        self.drawTrail((200,100,100))


class Robot:
    # TODO: Add starting direction theta as an init parameter
    def __init__(self, startpos, sensors=None):
        
        # Simplified robot
        self.B = startpos            # center point of robot
        self.x, self.y = startpos
        self.width = 45              # width of robot : 45cm
        self.theta = 0               # direction of the robot

        self.u = 0                   # linear velocity of robot
        self.w = 0                   # angular velocity of robot
        # LIMITS
        self.v_max = 250             # Max linear speed (pixels per second)
        self.w_max = 5               # Max angular speed (radians per second)

        self.a = 35                  # look ahead distance

        self.path = []
        self.waypoint = 0

        self.vl = 0                  # velocity of left wheel 
        self.vr = 0                  # velocity of right wheel 

        # Integration with sensors
        self.sensors = sensors
        self.sensor_offsets = []
        if self.sensors:
            for sensor in self.sensors:
                # Calculate relative position (local x, local y)
                rel_x = sensor.x - self.x
                rel_y = sensor.y - self.y
                # Store the relative heading 
                rel_heading = sensor.heading - self.theta
                self.sensor_offsets.append((rel_x, rel_y, rel_heading))

        # # ===================== Parameters for motor driving-based movement ================================= 
        # # Kinematics variables
        # self.C = 0                                  # instantaneous center of rotation
        # self.omega = 0                              # instantaneous angular velocity of the robot body
        # self.L = (self.x - self.s/2, self.y)        # center point of left wheel
        # self.R = (self.x + self.s/2, self.y)        # center point of right wheel
        # self.vb = 0                                 # velocity of the center point B
        # self.phiL = 0                               # angular velocity of left wheel
        # self.phiR = 0                               # angular velocity of right wheel
        # self.dist_l = abs(self.C - self.L)          # distance between C and L
        # self.r = 10                                 # radius of the wheels : 10cm
        # self.x_dot = 0                              # projection of vb onto the x-axis
        # self.y_dot = 0                              # projection of vb onto the y-axis
        # self.dist_l = 0                             # distance traveled by left wheel since last time step
        # self.dist_r = 0                             # distance traveled by right wheel since last time step
        # self.dist_center = 0                        # distance traveled by center of robot
        # # Odometry
        # self.TicksPerRotation = 384.5
        # self.voltageR = 0
        # self.voltageL = 0

       
    # Direct model
    # control movement of robot using linear and angular velocities
    def move(self, dt):
        # update robot x,y position based on linear and angular velocities
        self.x += (self.u * math.cos(self.theta) - self.a * math.sin(self.theta) * self.w) * dt
        self.y += (self.u * math.sin(self.theta) + self.a * math.cos(self.theta) * self.w) * dt
        
        # update robot heading based on angular velocity
        self.theta += self.w * dt
        if self.theta >= 2*math.pi or self.theta <= -2*math.pi:
            self.theta = 0
    
    def moveSensors(self):
        for i, sensor in enumerate(self.sensors):
            rel_x, rel_y, rel_heading = self.sensor_offsets[i]
            
            # Rotate the relative position vector by the robot's current theta
            # Standard rotation matrix:
            # x' = x*cos(theta) - y*sin(theta)
            # y' = x*sin(theta) + y*cos(theta)
            rotated_x = rel_x * math.cos(self.theta) - rel_y * math.sin(self.theta)
            rotated_y = rel_x * math.sin(self.theta) + rel_y * math.cos(self.theta)

            # Update sensor absolute position
            sensor.x = self.x + rotated_x
            sensor.y = self.y + rotated_y
            
            # Update sensor absolute heading
            sensor.heading = rel_heading - self.theta
    
    def moveRobot(self, dt):
        self.move(dt)
        if self.sensors:
            self.moveSensors()
        self.follow_path()   

    # Inverse model
    # get velocities for move() from waypoint coordinates
    def follow_path(self):
        if len(self.path)==0:
            return 
        
        # current point to get to
        target = self.path[self.waypoint]

        # distance vector to the target
        d_x = target[0] - self.x
        d_y = target[1] - self.y

        # dot product between distance vector and robot direction
        self.u = d_x * math.cos(self.theta) + d_y * math.sin(self.theta)

        # cross product scaled by distance to target to set turning speed 
        self.w = (-1/self.a) * math.sin(self.theta) * d_x + (1/self.a) * math.cos(self.theta) * d_y

        # CLAMP LINEAR VELOCITY (u)
        if self.u > self.v_max:
            self.u = self.v_max
        elif self.u < -self.v_max:
            self.u = -self.v_max
        # CLAMP ANGULAR VELOCITY (w)
        if self.w > self.w_max:
            self.w = self.w_max
        elif self.w < -self.w_max:
            self.w = -self.w_max

        # move to next point if within a certain range
        dist2waypoint = self.distance((self.x, self.y), self.path[self.waypoint])
        if dist2waypoint <= 35:
            self.waypoint += 1
        if self.waypoint >= len(self.path):
            self.waypoint = len(self.path) - 1
            if dist2waypoint < 1:
                # stop robot if end reached
                self.w = 0
                self.u = 0

    def distance(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        px = (float(x2) - float(x1))**2 
        py = (float(y2) - float(y1))**2
        return math.sqrt(px + py)


    # def drive(self, vl, vr):
    #     self.vl = vl
    #     self.vr = vr

    # def update_robotPos(self, dt):
    #     # Linear and Angular velocities
    #     v = (self.vl + self.vr) / 2
    #     omega = (self.vr - self.vl) / self.width

    #     # Update position
    #     self.x += v * math.cos(self.theta) * dt
    #     self.y -= v * math.sin(self.theta) * dt
    #     self.theta += (omega * dt)
        
    #     # Bind theta
    #     if abs(self.theta) > 2 * math.pi:
    #         self.theta = 0