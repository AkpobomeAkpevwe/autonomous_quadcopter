import numpy as np
import matplotlib.pyplot as plt
from ubicoders_vrobots import VRobotGraphClient, VRDataWriter, VRobotState, VirtualRobot, vr_client_main
np.set_printoptions(suppress=True, precision=2)

class PIController:
    def __init__(self, kp=0, ki=0) -> None:
        self.sp = 0 # setpoint
        self.error = 0 # error
        self.error_integral = 0 # integral error
        self.kp = kp # proportional gain
        self.ki = ki # integral gain
    
    def update(self, sp, current_state):
        self.error = sp - current_state  #error calculation
        self.error_integral += self.error # integral error calculation
        self.error_integral = np.clip(self.error_integral, -100, 100) # integral error clipping
        cmd_out = self.error*self.kp + self.error_integral*self.ki # output command
        return cmd_out


class MyVRobotsController:
    def __init__(self, vr: VirtualRobot = None):
        self.vr = vr
        self.pi_attitude = PIController(0.1, 0)
        self.pi_rate = PIController(50 ,0.9)
        
    def loop(self):
        # Retrieve the states
        states: VRobotState = self.vr.states

        # Reading the data - time stamp           
        ts = states.timestamp/1000 # seconds.
        print(f"ts: {ts}")

        euler = states.euler.x
        print(f"euler: {euler}")

        angvel = states.angVel.x
        print(f"angvel: {angvel}")

        # DO SOME COOL STUFF HERE
        rate_sp = self.pi_attitude.update(0, euler)
        rate_out = self.pi_rate.update(rate_sp, angvel)

        throttle = 1500
        pwm_left = throttle + rate_out
        pwm_right = throttle - rate_out

        # Update Actuators
        self.vr.update_cmd_multirotor(0, [pwm_left, pwm_right, 0, 0])


# Main code
if __name__ == "__main__":
    # Instantiate the controller
    controller = MyVRobotsController()

    # Run the controller without modifying your main loop
    vr_client_main(controller, duration=5)  # Interact for 60 seconds    
    
    print("simulation finished")




