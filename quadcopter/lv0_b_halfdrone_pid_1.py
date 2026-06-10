import numpy as np
import matplotlib.pyplot as plt
from ubicoders_vrobots import VRobotGraphClient, VRDataWriter, VRobotState, VirtualRobot, vr_client_main
np.set_printoptions(suppress=True, precision=2)

class MyVRobotsController:
    def __init__(self, vr: VirtualRobot = None):
        self.vr = vr
        self.sp = 0
        self.kp =15
        self.kd = 100
        self.ki = 1.6
        self.error_integral = 0
        
    def loop(self):
        # Retrieve the states
        states: VRobotState = self.vr.states
        ts = states.timestamp/1000 # seconds.
        eulerX = states.euler.x
        angVelX = states.angVel.x

        # DO SOME COOL STUFF HERE
        error = self.sp - eulerX
        d_error = angVelX

        control_cmd = self.kp*error - self.kd*d_error + self.ki*self.error_integral

        throttle = 1500

        left_pwm = throttle + control_cmd
        right_pwm = throttle - control_cmd


        # Update Actuators
        self.vr.update_cmd_multirotor(0, [left_pwm, right_pwm, 0, 0])


# Main code
if __name__ == "__main__":
    # Instantiate the controller
    controller = MyVRobotsController()

    # Run the controller without modifying your main loop
    vr_client_main(controller, duration=30)  # Interact for 60 seconds    
    
    print("simulation finished")
