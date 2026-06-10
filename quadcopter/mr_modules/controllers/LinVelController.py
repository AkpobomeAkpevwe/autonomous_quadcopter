import numpy as np
from mr_modules.controllers.PIController import PIControllerSingleAxis

class LinVelController:
    def __init__(self):
        self.vel_x = PIControllerSingleAxis(kp=0.1, ki=0, error_max=1)
        self.vel_y = PIControllerSingleAxis(kp=0.1, ki=0, error_max=1)
        self.vel_z = PIControllerSingleAxis(kp=100, ki=100, error_max=1)

    def update(self, error):
        velx_error, vely_error, velz_error = error
        velx_cmd = self.vel_x.update(velx_error)
        vely_cmd = self.vel_y.update(vely_error)
        velz_cmd = self.vel_z.update(velz_error)
        cmd_vel = np.array([velx_cmd, vely_cmd, velz_cmd])
        cmd_vel = self.clamp_vel_sp(cmd_vel)
        return cmd_vel
    
    def clamp_vel_sp(self, vel_sp):
        lim_xy = 30 * np.pi/180
        vel_sp[0] = np.clip(vel_sp[0], -lim_xy, lim_xy) 
        vel_sp[1] = np.clip(vel_sp[1], -lim_xy, lim_xy)
        return vel_sp