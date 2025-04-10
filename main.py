import numpy as np
import matplotlib.pyplot as plt
from utils.Robot import Robot
from utils.GoalPlanner import GoalPlanner
    
robot = Robot(0, 0, 0)
goalPlanner = GoalPlanner(-5, -5, -90, robot)
goalPlanner.setParameter(3, 8, -1.5)

total_time = 100
dt = 0.01

x_path = []
y_path = []
theta_path = []

robot.drawRobot()

for t in np.arange(0, total_time, dt):
    x_path.append(robot.x)
    y_path.append(robot.y)
    v, w = goalPlanner.calculateVelocity()
    robot.updatePoseByVelocity(v, w, dt)
    theta_path.append(robot.theta)

robot.drawRobot(color = 'b')



plt.plot(x_path, y_path)
plt.xlabel('x Position')
plt.ylabel('y Position')
plt.title('Robot Trajectory')
plt.axis('equal')
plt.grid()
plt.show()