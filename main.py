import math
import pygame
from robot_1 import Graphics,Robot,Ultrasonic

MAP_DIMENTIONS = (600,1200)

gfx = Graphics(MAP_DIMENTIONS,'robot.png','room.png')

start = (1085,65)
ignored_area = (1055, 8, 1119, 60)
robot = Robot(start,0.01 * 3779.52)

sensor_range = 60, math.radians(45)

ultra_sonic = Ultrasonic(sensor_range,gfx.map)

dt =0
last_time = pygame.time.get_ticks()

running = True
trail = []
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                robot.acceleration_factor += 1
            if event.key == pygame.K_DOWN:
                robot.acceleration_factor -= 1

    dt = (pygame.time.get_ticks() - last_time) / 1000
    last_time = pygame.time.get_ticks()
    gfx.map.fill((255, 255, 255))
    gfx.map.blit(gfx.map_img,(0,0))
    robot.update_charge()
    robot.kinematics(dt)
    gfx.draw_robot(robot.x,robot.y,robot.heading)
    trail.append((robot.x, robot.y))

    point_cloud = ultra_sonic.sence_obstacles(robot.x, robot.y, robot.heading, ignored_area)
    robot.avoid_obstacles(point_cloud,dt)
    gfx.draw_sensor_data(point_cloud)

    for i in range(len(trail) - 1):
        pygame.draw.line(gfx.map, (0, 255, 0), trail[i], trail[i + 1], 35)

    pygame.display.update()