import numpy as np
from mr_modules.controllers.PIController import PIControllerSingleAxis

# Angular Rate Controller
class RateController:
    def __init__(self):
        self.roll_rate_ctrl = PIControllerSingleAxis(kp=50, ki=0, error_max=1)
        self.pitch_rate_ctrl = PIControllerSingleAxis(kp=50, ki=0, error_max=1)
        self.yaw_rate_ctrl = PIControllerSingleAxis(kp=10, ki=0, error_max=1)

    def update(self, error):
        roll_rate_error, pitch_rate_error, yaw_rate_error = error
        roll_rate_cmd = self.roll_rate_ctrl.update(roll_rate_error)
        pitch_rate_cmd = self.pitch_rate_ctrl.update(pitch_rate_error)
        yaw_rate_cmd = self.yaw_rate_ctrl.update(yaw_rate_error)
        rate_cmd = np.array([roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd])
        return rate_cmd