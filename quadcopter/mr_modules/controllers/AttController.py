import numpy as np
from mr_modules.controllers.PIController import PIControllerSingleAxis

# Attitude Controller
class AttController:
    def __init__(self):
        self.roll_ctrl = PIControllerSingleAxis(kp=1, ki=0, error_max=1)
        self.pitch_ctrl = PIControllerSingleAxis(kp=1, ki=0, error_max=1)
        self.yaw_ctrl = PIControllerSingleAxis(kp=0.5, ki=0, error_max=1)

    def update(self, error):
        roll_error, pitch_error, yaw_error = error
        roll_cmd = self.roll_ctrl.update(roll_error)
        pitch_cmd = self.pitch_ctrl.update(pitch_error)
        yaw_cmd = self.yaw_ctrl.update(yaw_error)
        att_cmd = np.array([roll_cmd, pitch_cmd, yaw_cmd])
        return att_cmd