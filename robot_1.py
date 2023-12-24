import pygame
import math
import numpy as np
import random
def distance(point1,point2):
    point1 = np.array(point1)
    point2 = np.array(point2)
    return np.linalg.norm(point1-point2)

class Robot:
    def __init__(self, startpos, width):
        self.m2p = 3779.52
        self.w = width
        self.charge = 100
        self.x = startpos[0]
        self.y = startpos[1]
        self.heading = 0

        self.vl = 0.01 * self.m2p
        self.vr = 0.01 * self.m2p

        self.maxspeed = 0.02 * self.m2p
        self.minspeed = 0.01 * self.m2p

        self.min_obs_dist = 30
        self.count_down = 5
        self.acceleration_factor = 1.0


    def update_charge(self):
        if pygame.time.get_ticks() % 1000 == 0:
            self.charge -= 1
            print("Charge level:", self.charge)

    def avoid_obstacles(self, point_cloud, dt):
        closest_obs = None
        dist = np.inf
        self.update_charge()
        if len(point_cloud) > 1:
            for point in point_cloud:
                if dist > distance([self.x, self.y], point):
                    dist = distance([self.x, self.y], point)
                    closest_obs = (point, dist)
                if closest_obs[1] < self.min_obs_dist and self.count_down > 0:
                    self.count_down -= dt
                    self.vl = 0
                    self.vr = 0
                    self.heading -= np.radians(random.randint(5,30))
                else:
                    self.count_down = 5
                    self.move_forward()


    def move_forward(self):
        self.vr = self.minspeed
        self.vl = self.minspeed

    def kinematics(self, dt):

        self.x += ((self.vl + self.vr) / 2) * math.sin(self.heading) * dt * self.acceleration_factor
        self.y += ((self.vl + self.vr) / 2) * math.cos(self.heading) * dt * self.acceleration_factor
        self.heading += (self.vr - self.vl) / self.w * dt
        if self.heading > 2 * math.pi or self.heading < -2 * math.pi:
            self.heading = 0

        self.vr = max(min(self.maxspeed, self.vr), self.minspeed)
        self.vl = max(min(self.maxspeed, self.vl), self.minspeed)


class Graphics:
    def __init__(self,dimentions,robot_png_path,map_img_path):
        pygame.init()
        self.black = (0,0,0)
        self.white = (255,255,255)
        self.green = (0,255,0)
        self.blue = (0,0,255)
        self.red = (255, 0, 0)
        self.yellow = (255, 255, 0)

        self.robot = pygame.image.load(robot_png_path)
        self.map_img = pygame.image.load(map_img_path)

        self.height, self.width = dimentions

        pygame.display.set_caption("My_project")
        self.map = pygame.display.set_mode([self.width,self.height])
        self.map.blit(self.map_img,(0,0))

    def draw_robot(self,x,y, heading):
        rotated = pygame.transform.rotozoom(self.robot,math.degrees(heading),1)
        rect = rotated.get_rect(center = (x,y))
        self.map.blit(rotated,rect)

    def draw_sensor_data(self,point_cloud):
        for point in point_cloud:
            pygame.draw.circle(self.map,self.red,point,3,0)

class Ultrasonic:

    def __init__(self,sensor_range, map):
        self.sensor_range = sensor_range
        self.map_width, self.map_height = pygame.display.get_surface().get_size()
        self.map = map

    def sence_obstacles(self,x,y,heading,ignored_area=None):
        obstacles = []
        x1, y1 = x, y
        start_angle = heading - self.sensor_range[1] - 1.5
        finish_angle = heading + self.sensor_range[1] - 1.5
        for angle in np.linspace(start_angle,finish_angle,10,False):
            x2 = x1 + self.sensor_range[0] * math.cos(angle)
            y2 = y1 - self.sensor_range[0] * math.sin(angle)
            for i in range (100):
                u = i/100
                x = int(x2 * u + x1 * (1-u))
                y = int(y2 * u + y1 * (1 - u))
                if 0 < x < self.map_width and 0 < y < self.map_height:
                    color = self.map.get_at((x,y))
                    self.map.set_at((x,y),(0,208,255))

                    if ignored_area is not None and ignored_area[0] <= x <= ignored_area[2] and \
                            ignored_area[1] <= y <= ignored_area[3]:
                        continue

                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        obstacles.append([x, y])
                        break
        return obstacles











