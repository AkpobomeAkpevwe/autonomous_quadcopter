import numpy as np
import matplotlib.pyplot as plt
from ubicoders_vrobots import VRobotGraphClient, VRDataWriter, VRobotState, VirtualRobot, vr_client_main
from lv3_invpen_env import InvPenEnvironment
np.set_printoptions(suppress=True, precision=2)

m, M, L, g = 0.2, 0.5, 2, -9.81
ic = np.array([-3, 0, np.pi*0.7, 0.5]) # initial states
invpen = InvPenEnvironment(ic, m=m, M=M, L=L, g=g)
#K = np.array([[-10., -19.74848692, 149.11334191, 67.0328901 ]])
K = np.array([[-1.,-2.37857572, 29.71263811, 12.53052112]])
target = np.array([2, 0, np.pi, 0])

class MyVRobotsController:
    def __init__(self, vr: VirtualRobot = None):
        self.vr = vr
        self.idx = 0
        self.prev_ts = 0
        self.dt = 0
        self.xnow = np.array([0, 0, 0, 0])

    def calc_dt(self, ts):
        if self.prev_ts == 0:
            self.prev_ts = ts
            return False
        else:
            self.dt = ts - self.prev_ts
            self.prev_ts = ts
            return True
        
    def loop(self):
        states: VRobotState = self.vr.states
        ts = states.timestamp/1000 # seconds.
        if (self.calc_dt(ts) == False):
            return        

        error = target - self.xnow
        u = K@error
        u = u[0]
        self.xnow = invpen.get_next(u, self.dt)
        
        pos = self.xnow[0]
        vel = self.xnow[1]
        ang = self.xnow[2]
        angvel = self.xnow[3]
        # Update Actuators
        self.vr.update_cmd_invpen(0, pos, vel, ang, angvel)


# Main code
if __name__ == "__main__":
    # Instantiate the controller
    controller = MyVRobotsController()

    # Run the controller without modifying your main loop
    vr_client_main(controller, duration=30)  # Interact for 60 seconds    
    
    print("simulation finished")
