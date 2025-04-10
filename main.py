import numpy as np
import matplotlib.pyplot as plt
from utils.Robot import Robot
from utils.GoalPlanner import GoalPlanner

radius = 5
robot_count = 8

goal_x = 0
goal_y = 0
goal_theta = 90

angles = np.linspace(0, 2*np.pi, robot_count, endpoint=False)

robots = []
planners = []

for angle in angles:
    start_x = radius * np.cos(angle)
    start_y = radius * np.sin(angle)
    robot = Robot(start_x, start_y, 0)
    planner = GoalPlanner(goal_x, goal_y, goal_theta, robot)
    planner.setParameter(3, 8, -1.5)

    robots.append(robot)
    planners.append(planner)

total_time = 10
dt = 0.01

x_paths = [[] for _ in range(robot_count)]
y_paths = [[] for _ in range(robot_count)]

for robot in robots:
    robot.drawRobot()

for t in np.arange(0, total_time, dt):
    for i in range(robot_count):
        robot = robots[i]
        planner = planners[i]
        
        v, w = planner.calculateVelocity()
        robot.updatePoseByVelocity(v, w, dt)

        x_paths[i].append(robot.x)
        y_paths[i].append(robot.y)

robot = Robot(0, 0, 0)
robot.drawRobot(color='b')

for i in range(robot_count):
    plt.plot(x_paths[i], y_paths[i])

plt.xlabel('x Position')
plt.ylabel('y Position')
plt.title('Multi-Robot Trajectory')
plt.axis('equal')
plt.xlim(-8, 8)
plt.ylim(-8, 8)
plt.grid()
plt.show()