import numpy as np
import matplotlib.pyplot as plt
from ubicoders_vrobots import VRobotGraphClient, VRDataWriter, VRobotState, VirtualRobot, vr_client_main
np.set_printoptions(suppress=True, precision=2)


graph_tool = VRobotGraphClient()


def lift_to_pwm_simple(lift):
    lift_sqrt = np.sqrt(lift)   
    return 364.5*lift_sqrt + 945

class MyVRobotsController:
    def __init__(self, vr: VirtualRobot = None):
        self.vr = vr
        self.K = np.array([[2.,  3]])
        self.target_z = np.array([-120.34, 0])
        self.w = 9.81*1
        
    def loop(self):
        # Retrieve the states
        states: VRobotState = self.vr.states

        # Reading the data - time stamp           
        ts = states.timestamp/1000 # seconds.
        print(f"ts: {ts}")

        # DO SOME COOL STUFF HERE
        posz = states.linPos.z
        velz = states.linVel.z
        current_state = np.array([posz, velz])
        err_pos_z = self.target_z - current_state
        err_pos_z[0] = np.clip(err_pos_z[0], -10, 10)
        u = (self.K @ err_pos_z)[0]
        u = np.clip(u, -26, 9)
        req_throtle = lift_to_pwm_simple((self.w-u)/4)
        req_throtle = np.clip(req_throtle, 1200, 1650)

        # graph rt
        graph_tool.update(ts, x = posz, y = velz, z = req_throtle)

        # Update Actuators
        self.vr.update_cmd_multirotor(0, [req_throtle, req_throtle, req_throtle, req_throtle])


# Main code
if __name__ == "__main__":
    # Instantiate the controller
    controller = MyVRobotsController()

    # Run the controller without modifying your main loop
    vr_client_main(controller, duration=50)  # Interact for 60 seconds    
    
    print("simulation finished")
