import numpy as np

class PIControllerSingleAxis:
    def __init__(self, kp=1, ki=1, error_max=100):
        self.kp = kp
        self.ki = ki
        self.integral = 0
        self.error_max = error_max

    def error_integrator(self, error):
        self.integral += error
        self.integral = np.clip(self.integral, -self.error_max, self.error_max)
    
    def update(self, error):
        self.error_integrator(error)
        return self.kp * error + self.ki * self.integral
