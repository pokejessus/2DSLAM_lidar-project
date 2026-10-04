import pygame
import math

class Robot:
    def __init__(self, start_x, start_y, start_angle=0, move_speed=2.0, turn_speed=5.0):
        # Fallback in case 'robot.png' is missing during your testing
        try:
            self.original_image = pygame.image.load('robot.png').convert_alpha()
        except pygame.error:
            print("Warning: 'robot.png' not found. Using a fallback red triangle.")
            self.original_image = pygame.Surface((40, 40), pygame.SRCALPHA)
            # Drawing a triangle pointing right (0 degrees)
            pygame.draw.polygon(self.original_image, (255, 0, 0),[(0, 10), (0, 30), (40, 20)])
        
        self.image = self.original_image
        self.rect = self.image.get_rect(center=(start_x, start_y))
        
        # Storing floats for accurate movement calculation
        self.x = float(start_x)
        self.y = float(start_y)
        self.angle = float(start_angle)
        
        self.path =[]
        self.current_target = None
        
        # Pixels and degrees per frame
        self.move_speed = move_speed  
        self.turn_speed = turn_speed  

    def set_path(self, new_path):
        """
        Receives the path from the algorithm.
        Format expected: list of tuples (x, y, distance, angle_in_degrees)
        """
        self.path = new_path
        # If we have a path but no active target, grab the first point
        if self.path and self.current_target is None:
            self.current_target = self.path.pop(0)

    def update(self):
        """
        Moves and turns the robot towards the current target in the path.
        Should be called once per frame.
        """
        if self.current_target is None:
            # Try to get the next step in the path
            if self.path:
                self.current_target = self.path.pop(0)
            else:
                return # No path to follow, robot stays still

        # Unpack the tuple
        target_x, target_y, target_distance, target_angle = self.current_target

        # --- 1. HANDLE ROTATION ---
        # Calculate the shortest turn direction (keeps angle difference between -180 and +180)
        angle_diff = (target_angle - self.angle) % 360
        if angle_diff > 180:
            angle_diff -= 360

        # Snap to angle if close enough, else smoothly turn
        if abs(angle_diff) <= self.turn_speed:
            self.angle = target_angle
        else:
            self.angle += self.turn_speed if angle_diff > 0 else -self.turn_speed

        self.angle %= 360

        # --- 2. HANDLE MOVEMENT ---
        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)

        # Snap to position if close enough, else smoothly move
        if dist <= self.move_speed:
            self.x = target_x
            self.y = target_y
            # Reached this step, clear target so next frame grabs the next point
            self.current_target = None 
        else:
            self.x += (dx / dist) * self.move_speed
            self.y += (dy / dist) * self.move_speed

        # --- 3. APPLY ROTATION AND POSITION TO IMAGE ---
        # Pygame rotates counter-clockwise. Positive angle = CCW.
        self.image = pygame.transform.rotate(self.original_image, self.angle)
        self.rect = self.image.get_rect(center=(round(self.x), round(self.y)))

    def draw(self, surface):
        """ Draws the robot onto the given Pygame screen/surface. """
        surface.blit(self.image, self.rect.topleft)