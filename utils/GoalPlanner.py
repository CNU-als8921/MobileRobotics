import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from Robot import Robot

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

        self.mode = "FORWARD"

    def setDirection(self):
        dx = self.goal_x - self.robot.x
        dy = self.goal_y - self.robot.y

        self.alpha = self.saturationRad(np.arctan2(dy, dx) - np.deg2rad(self.robot.theta))

        if(abs(self.alpha) > np.pi / 2): 
            self.mode = "REVERSE"
            return "REVERSE"
        else: 
            self.mode = "FORWARD"
            return "FORWARD"

    def setParameter(self, k_r, k_a, k_b):
        self.K_rho = k_r
        self.K_alpha = k_a
        self.K_beta = k_b

    def calculateVelocity(self):
        dx = self.goal_x - self.robot.x
        dy = self.goal_y - self.robot.y

        path_theta = np.arctan2(dy, dx)

        if self.mode == "REVERSE":
            path_theta = self.saturationRad(path_theta + np.pi)

        self.rho = np.sqrt(dx ** 2 + dy ** 2)
        self.alpha = self.saturationRad(path_theta - np.deg2rad(self.robot.theta))
        self.beta = self.saturationRad(np.deg2rad(self.goal_theta) - path_theta)

        v = self.K_rho * self.rho
        w = self.K_alpha * self.alpha + self.K_beta * self.beta


        if self.mode == "REVERSE":
            v = -v

        if self.rho < 0.05:
            v = 0
            heading_error = self.saturationRad(np.deg2rad(self.goal_theta) - np.deg2rad(self.robot.theta))
            w = 1.0 * heading_error
    
        return v, w

    def saturationDeg(self, deg):
        return (deg + 180) % 360 - 180
    

    def saturationRad(self, rad):
        return (rad + np.pi) % (2 * np.pi) - np.pi