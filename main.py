import numpy as np
import matplotlib.pyplot as plt
from utils.Robot import Robot
from utils.GoalPlanner import GoalPlanner

radius = 5
robot_count = 8

goal_x = 0
goal_y = 0
goal_theta = 180

robots = []
planners = []

for angle in np.linspace(0, 2*np.pi, robot_count, endpoint=False):
    start_x = radius * np.cos(angle)
    start_y = radius * np.sin(angle)
    start_theta = 0
    robot = Robot(start_x, start_y, start_theta)
    planner = GoalPlanner(goal_x, goal_y, goal_theta, robot)
    planner.setParameter(3, 8, -1.5)
    planner.setDirection()
    robots.append(robot)
    planners.append(planner)

total_time = 10
dt = 0.01

x_paths = [[] for _ in range(robot_count)]
y_paths = [[] for _ in range(robot_count)]

for i in range(robot_count):
    if(planners[i].mode == "FORWARD"): robots[i].drawRobot(color = 'g')
    elif(planners[i].mode == "REVERSE"): robots[i].drawRobot(color = 'r')

for t in np.arange(0, total_time, dt):
    for i in range(robot_count):
        robot : Robot = robots[i]
        planner : GoalPlanner = planners[i]
        
        v, w = planner.calculateVelocity()
        robot.updatePoseByVelocity(v, w, dt)

        x_paths[i].append(robot.x)
        y_paths[i].append(robot.y)

robot = Robot(goal_x, goal_y, goal_theta)
robot.drawRobot(color = 'b')

for i in range(robot_count):
    plt.plot(x_paths[i], y_paths[i], color = [0, 0, 0, 0.5])

plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Multi-Robot Trajectory')
plt.axis('equal')
plt.xlim(-8, 8)
plt.ylim(-8, 8)
plt.grid()
plt.show()