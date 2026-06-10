import numpy as np
from mr_modules.controllers.PIController import PIControllerSingleAxis

class LinPosController:
    def __init__(self):
        self.pos_x = PIControllerSingleAxis(kp=0.25, ki=0, error_max=1)
        self.pos_y = PIControllerSingleAxis(kp=0.25, ki=0, error_max=1)
        self.pos_z = PIControllerSingleAxis(kp=0.5, ki=0.1, error_max=1)

    def update(self, error):
        posx_error, posy_error, posz_error = error
        posx_cmd = self.pos_x.update(posx_error)
        posy_cmd = self.pos_y.update(posy_error)
        posz_cmd = self.pos_z.update(posz_error)
        cmd_pos = np.array([posx_cmd, posy_cmd, posz_cmd])
        cmd_pos = self.clamp_vel_sp(cmd_pos)
        return cmd_pos
    
    def clamp_vel_sp(self, vel_sp):
        lim_xy = 5
        vel_sp[0] = np.clip(vel_sp[0], -lim_xy, lim_xy) 
        vel_sp[1] = np.clip(vel_sp[1], -lim_xy, lim_xy)
        return vel_sp
    
