import numpy as np
import matplotlib.pyplot as plt

class Robot:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def updatePoseByVelocity(self, v, w, dt):
        self.x += v * np.cos(np.deg2rad(self.theta)) * dt
        self.y += v * np.sin(np.deg2rad(self.theta)) * dt
        self.theta += np.rad2deg(w) * dt

    def drawRobot(self, line_length = 1, color = 'r'):

        dx = line_length * np.cos(np.deg2rad(robot.theta))
        dy = line_length * np.sin(np.deg2rad(robot.theta))

        plt.plot(self.x, self.y, 'o', markersize = 30, color=color, alpha = 0.3, zorder=5)
        plt.arrow(self.x, self.y, dx, dy, head_width=0, head_length=0, fc=color, ec=color,linewidth=1.5, zorder=5)
        
class GoalPlanner:
    def __init__(self, desX, desY, desTheta, robot : Robot):
        self.goal_x = desX
        self.goal_y = desY
        self.goal_theta = desTheta
        self.rho = 0
        self.alpha = 0
        self.beta = 0
        self.robot = robot

        self.K_rho = 0
        self.K_alpha = 0
        self.K_beta = 0

    def setParameter(self, k_r, k_a, k_b):
        self.K_rho = k_r
        self.K_alpha = k_a
        self.K_beta = k_b

    def calculateVelocity(self):
        dx = self.goal_x - self.robot.x
        dy = self.goal_y - self.robot.y

        self.rho = np.sqrt(dx ** 2 + dy ** 2)
        self.alpha = self.saturationRad(np.arctan2(dy, dx) - np.deg2rad(self.robot.theta))
        self.beta = self.saturationRad(np.deg2rad(self.goal_theta) - np.arctan2(dy, dx))

        v = self.K_rho * self.rho
        w = self.K_alpha * self.alpha + self.K_beta * self.beta
    
        return v, w

    def saturationDeg(self, deg):
        return (deg + 180) % 360 - 180
    

    def saturationRad(self, rad):
        return (rad + np.pi) % (2 * np.pi) - np.pi
    
robot = Robot(0, 0, 0)
goalPlanner = GoalPlanner(5, 5, -90, robot)
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