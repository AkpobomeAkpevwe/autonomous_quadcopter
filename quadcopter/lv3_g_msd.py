import numpy as np
import matplotlib.pyplot as plt
from ubicoders_vrobots import VRobotGraphClient, VRDataWriter, VRobotState, VirtualRobot, vr_client_main

rt = VRobotGraphClient()

A = np.array([[0, 1],
              [-3.1, -0.2]])

B = np.array([[0],[1]])

dt = 0.02


class MyVRobotsController:
    def __init__(self, vr: VirtualRobot = None):
        self.vr = vr
        self.state = np.array([10, 0.,])        
        
    def loop(self):
        vr_state:VRobotState = self.vr.states
        ts = vr_state.timestamp/1000
        xdot = A @ self.state
        self.state += xdot * dt
        pos = self.state[0]
        vel = self.state[1]
        self.vr.update_cmd_msd(0, pos)

        rt.update(ts, x=pos, y=vel)

# Main code
if __name__ == "__main__":
    # Instantiate the controller
    controller = MyVRobotsController()

    # Run the controller without modifying your main loop
    vr_client_main(controller, duration=50)  # Interact for 60 seconds    
    
    print("simulation finished")
