import numpy as np
import matplotlib.pyplot as plt
from ubicoders_vrobots import VRobotGraphClient, VRDataWriter, VRobotState, VirtualRobot, vr_client_main
np.set_printoptions(suppress=True, precision=2)

rtg = VRobotGraphClient()

dc = VRDataWriter("data.csv")

class MyVRobotsController:
    def __init__(self, vr: VirtualRobot = None):
        self.vr = vr
        
    def loop(self):
        # Retrieve the states
        states: VRobotState = self.vr.states

        dc.record(states)

        angvelX = states.angVel.x
        angvelY = states.angVel.y
        angvelZ = states.angVel.z

        print(f"angvelX: {angvelX}, angvelY: {angvelY}, angvelZ: {angvelZ}")



        # Reading the data - time stamp           
        ts = states.timestamp/1000 # seconds.
        print(f"ts: {ts}")


        rtg.update(ts, np.array([angvelX, angvelY, angvelZ]))

        # DO SOME COOL STUFF HERE


        # Update Actuators
        self.vr.update_cmd_multirotor(0, [1800, 1800, 0, 0])


# Main code
if __name__ == "__main__":
    # Instantiate the controller
    controller = MyVRobotsController()

    # Run the controller without modifying your main loop
    vr_client_main(controller, duration=5)  # Interact for 60 seconds    
    
    print("simulation finished")
